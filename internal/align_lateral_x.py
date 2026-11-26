import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from apriltag_msgs.msg import AprilTagDetectionArray
from tf2_ros import Buffer, TransformListener, TransformException
import math
import sys
import argparse
import time
import tf_transformations

class PrecisionCalibrationNode(Node):
    def __init__(self, target_frame='tag_0', yaw_tol=0.1, target_offset=1.7, precision=0.005):
        super().__init__('precision_calibration_node')
        
        # --- [Parameter Configuration] ---
        self.target_frame = target_frame
        self.source_frame = 'camera'
        self.target_offset_val = target_offset
        self.YAW_TOL = math.radians(yaw_tol)
        self.Y_PRECISION_TOL = precision  # Completed if within 0.5cm
        self.OFFSET = 0.39

        # [S30 Optimized Control Parameters]
        self.MAX_ANG_VEL = 0.25
        self.MIN_ANG_VEL = 0.12   # Minimum velocity to overcome friction
        self.KP_ANG = 1.2
        self.AVG_SIZE = 15        # Number of samples for data stabilization

        # --- [State Control] ---
        # Overall states: ALIGN_YAW -> MEASURE_Y -> ZIGZAG_CORRECT -> FINISHED
        self.state = 'ALIGN_YAW'
        # Sub-states within ALIGN_YAW: MEASURE -> ROTATE -> STOP_WAIT
        self.align_sub_state = 'MEASURE'
        
        self.yaw_history = []
        self.y_measurements = []
        self.avg_dist_y = 0.0
        self.current_avg_yaw = 0.0
        
        self.sub_mission_queue = []
        self.current_sub_task = None
        self.task_initialized = False
        self.move_start_time = None
        self.is_tag_visible = False

        # Communication and Timers
        self.publisher = self.create_publisher(Twist, '/mobile_base/cmd_vel', 10)
        self.detection_sub = self.create_subscription(
            AprilTagDetectionArray, '/detections', self.detection_callback, 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 20Hz control loop (considering S30 communication stability)
        self.timer = self.create_timer(1.0/20.0, self.control_loop)

        self.get_logger().info(f"🚀 Precision calibration started: YawTol={yaw_tol}deg, TargetOffset={target_offset}")

    def detection_callback(self, msg):
        try:
            target_id = int(self.target_frame.split('_')[-1])
        except ValueError:
            target_id = 0
        self.is_tag_visible = any(d.id == target_id for d in msg.detections)

    def get_tag_info(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.source_frame,
                self.target_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05))
            
            cam_z = t.transform.translation.z
            cam_x = t.transform.translation.x
            q = t.transform.rotation
            
            _, raw_yaw, _ = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
            
            # Yaw normalization and offset application
            yaw_deg = math.degrees(raw_yaw) - self.target_offset_val
            if yaw_deg > 90: yaw_deg -= 180
            elif yaw_deg < -90: yaw_deg += 180
            
            current_yaw_rad = math.radians(yaw_deg)
            dist_x = cam_z + (math.cos(current_yaw_rad) * self.OFFSET)
            dist_y = (-cam_x) + (math.sin(current_yaw_rad) * self.OFFSET)
            
            return dist_x, dist_y, current_yaw_rad
        except TransformException:
            return None, None, None

    def stop_robot(self):
        self.publisher.publish(Twist())

    def control_loop(self):
        if not self.is_tag_visible and self.state != 'ZIGZAG_CORRECT':
            self.stop_robot()
            return

        dx, dy, current_yaw_rad = self.get_tag_info()
        now_time = self.get_clock().now()

        # 1. Align front angle (applied the suggested new precision rotation logic)
        if self.state == 'ALIGN_YAW':
            if current_yaw_rad is None: return

            if self.align_sub_state == "MEASURE":
                self.stop_robot()
                self.yaw_history.append(current_yaw_rad)
                
                if len(self.yaw_history) >= self.AVG_SIZE:
                    self.current_avg_yaw = sum(self.yaw_history) / len(self.yaw_history)
                    self.yaw_history = []
                    
                    if abs(self.current_avg_yaw) <= self.YAW_TOL:
                        self.get_logger().info("🎯 Angle alignment completed -> Entering Y deviation measurement")
                        self.state = "MEASURE_Y"
                        self.align_sub_state = "MEASURE" # Initialization
                    else:
                        self.align_sub_state = "ROTATE"
                        self.move_start_time = now_time

            elif self.align_sub_state == "ROTATE":
                twist = Twist()
                target_ang = self.current_avg_yaw * -self.KP_ANG
                
                # Velocity clipping
                limited_ang = math.copysign(
                    max(self.MIN_ANG_VEL, min(abs(target_ang), self.MAX_ANG_VEL)), 
                    target_ang
                )
                
                # Apply differential operation time based on error
                move_duration = 0.4 if abs(self.current_avg_yaw) > math.radians(1.0) else 0.25
                elapsed = (now_time - self.move_start_time).nanoseconds / 1e9

                if elapsed < move_duration:
                    twist.angular.z = limited_ang
                    self.publisher.publish(twist)
                else:
                    self.stop_robot()
                    self.align_sub_state = "STOP_WAIT"
                    self.move_start_time = now_time

            elif self.align_sub_state == "STOP_WAIT":
                elapsed = (now_time - self.move_start_time).nanoseconds / 1e9
                if elapsed > 1.2: # Wait for inertia settling
                    self.align_sub_state = "MEASURE"

        # 2. Measure dist_y
        elif self.state == 'MEASURE_Y':
            if dy is None: return
            self.stop_robot()
            self.y_measurements.append(dy)
            if len(self.y_measurements) >= self.AVG_SIZE:
                self.avg_dist_y = sum(self.y_measurements) / len(self.y_measurements)
                self.y_measurements = []
                self.get_logger().info(f"📊 Average Y deviation: {self.avg_dist_y:.4f}m")
                
                if abs(self.avg_dist_y) <= self.Y_PRECISION_TOL:
                    self.get_logger().info("🎯 Final calibration successful!")
                    self.state = 'FINISHED'
                else:
                    self.plan_zigzag_mission(self.avg_dist_y)
                    self.state = 'ZIGZAG_CORRECT'

        # 3. Zigzag correction driving
        elif self.state == 'ZIGZAG_CORRECT':
            self.execute_sub_mission()

        elif self.state == 'FINISHED':
            self.stop_robot()
            raise SystemExit

    def plan_zigzag_mission(self, error_y):
        # Calculate angle and travel distance based on error magnitude
        angle = math.radians(10.0)
        if abs(error_y) > 0.6: angle = math.radians(20.0)
        if abs(error_y) < 0.3: angle = math.radians(5.0)

        move_dist = abs(error_y) / math.sin(angle)
        # Distance clamping
        if move_dist > 0.5: move_dist = 0.5
        if abs(error_y) < 0.2: move_dist = max(abs(error_y)*2., 0.05)

        turn_dir = -1.0 if error_y > 0 else 1.0

        self.sub_mission_queue = [
            ('ROTATE', turn_dir * angle),
            ('FORWARD', -move_dist),
            ('ROTATE', -turn_dir * angle * 2.0),
            ('FORWARD', move_dist),
            ('ROTATE', turn_dir * angle)
        ]
        self.current_sub_task = None
        self.get_logger().info(f"🔄 Generated zigzag sequence (Travel distance: {move_dist:.3f}m)")

    def execute_sub_mission(self):
        if self.current_sub_task is None:
            if self.sub_mission_queue:
                self.current_sub_task = self.sub_mission_queue.pop(0)
                self.task_initialized = False
                return
            else:
                self.state = 'ALIGN_YAW' # Return to angle alignment after one set completes
                self.align_sub_state = 'MEASURE'
                return

        task_type, val = self.current_sub_task
        twist = Twist()

        if not self.task_initialized:
            self.task_start_clock = self.get_clock().now()
            if task_type == 'FORWARD': 
                self.duration = abs(val) / 0.1
            else: 
                self.duration = abs(val) / 0.25 # Angular velocity based on S30 standard
            self.task_initialized = True

        elapsed = (self.get_clock().now() - self.task_start_clock).nanoseconds / 1e9
        if elapsed < self.duration:
            if task_type == 'FORWARD':
                twist.linear.x = 0.1 if val > 0 else -0.1
            else:
                twist.angular.z = 0.25 if val > 0 else -0.25
            self.publisher.publish(twist)
        else:
            self.stop_robot()
            self.current_sub_task = None

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--target_frame', type=str, default='tag_0')
    parser.add_argument('--yaw_tol', type=float, default=0.1)
    parser.add_argument('--target', type=float, default=1.7)
    parser.add_argument('--precision', type=float, default=0.005)
    
    args = parser.parse_args(rclpy.utilities.remove_ros_args(args=sys.argv)[1:])

    rclpy.init()
    node = PrecisionCalibrationNode(args.target_frame, args.yaw_tol, args.target, args.precision)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if rclpy.ok():
            node.stop_robot()
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
