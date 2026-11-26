import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from apriltag_msgs.msg import AprilTagDetectionArray
from tf2_ros import Buffer, TransformListener, TransformException
import math
import sys
import argparse
import tf_transformations

class PrecisionTargetRotateNode(Node):
    def __init__(self, target_frame='tag_0', yaw_tol=0.1, target_offset=1.7):
        super().__init__('precision_target_rotate_node')
        
        # --- [S30 Optimized Configuration Values] ---
        self.target_frame = target_frame
        self.source_frame = 'camera' 
        self.target_offset = target_offset
        self.YAW_TOL = math.radians(yaw_tol) 
        
        # S30 hardware characteristics: Velocity settings to overcome rotational friction
        self.MAX_ANG_VEL = 0.25   # Too fast may cause overshoot
        self.MIN_ANG_VEL = 0.12   # Minimum velocity required for S30 to rotate (Important)
        self.KP_ANG = 1.2         # Proportional control constant for angle

        # --- [State and Measurement Management] ---
        self.yaw_history = []
        self.AVG_SIZE = 15        # Increased sample size to filter out S30 micro-vibrations
        self.state = "MEASURE"    # MEASURE, ROTATE, STOP_WAIT
        self.current_avg_yaw = 0.0
        self.move_start_time = None

        self.is_tag_visible = False

        self.publisher = self.create_publisher(Twist, '/mobile_base/cmd_vel', 10)
        self.detection_sub = self.create_subscription(
            AprilTagDetectionArray, '/detections', self.detection_callback, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 20Hz control loop (for S30 communication stability)
        self.timer = self.create_timer(1.0/20.0, self.control_loop)
        self.get_logger().info(f"🚀 S30 precision angle adjustment started: Target tolerance={yaw_tol}deg")

    def detection_callback(self, msg):
        try:
            target_id = int(self.target_frame.split('_')[-1])
        except ValueError:
            target_id = 0
        self.is_tag_visible = any(d.id == target_id for d in msg.detections)

    def stop_robot(self):
        self.publisher.publish(Twist())

    def control_loop(self):
        if not self.is_tag_visible:
            self.stop_robot()
            return

        try:
            now_time = self.get_clock().now()
            # Lookup latest transform
            t = self.tf_buffer.lookup_transform(self.source_frame, self.target_frame, rclpy.time.Time())
            
            # Calculate and normalize Yaw
            q = t.transform.rotation
            _, raw_yaw, _ = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
            yaw_deg = math.degrees(raw_yaw) - self.target_offset
            if yaw_deg > 90: yaw_deg -= 180
            elif yaw_deg < -90: yaw_deg += 180
            current_yaw_rad = math.radians(yaw_deg)

            # --- [State Machine] ---

            if self.state == "MEASURE":
                self.stop_robot()
                self.yaw_history.append(current_yaw_rad)
                
                if len(self.yaw_history) >= self.AVG_SIZE:
                    self.current_avg_yaw = sum(self.yaw_history) / len(self.yaw_history)
                    self.yaw_history = []
                    
                    avg_deg = math.degrees(self.current_avg_yaw)
                    self.get_logger().info(f"📊 Measurement Average: {avg_deg:.3f}deg")

                    if abs(self.current_avg_yaw) <= self.YAW_TOL:
                        self.get_logger().info("🎯 Target angle reached successfully!")
                        raise SystemExit
                    else:
                        self.state = "ROTATE"
                        self.move_start_time = now_time

            elif self.state == "ROTATE":
                twist = Twist()
                # Calculate velocity based on P control
                target_ang = self.current_avg_yaw * -self.KP_ANG
                
                # Clip between minimum and maximum velocity
                limited_ang = math.copysign(
                    max(self.MIN_ANG_VEL, min(abs(target_ang), self.MAX_ANG_VEL)), 
                    target_ang
                )
                
                # Determine operation duration (0.4s for large errors, 0.25s for small errors)
                move_duration = 0.4 if abs(self.current_avg_yaw) > math.radians(1.0) else 0.25
                elapsed = (now_time - self.move_start_time).nanoseconds / 1e9

                if elapsed < move_duration:
                    twist.angular.z = limited_ang
                    self.publisher.publish(twist)
                else:
                    self.stop_robot()
                    self.state = "STOP_WAIT"
                    self.move_start_time = now_time # Reused for measuring wait time

            elif self.state == "STOP_WAIT":
                # Wait until S30 inertia settles (rotation has greater vibration than linear motion)
                elapsed = (now_time - self.move_start_time).nanoseconds / 1e9
                if elapsed > 1.2:
                    self.state = "MEASURE"

        except (TransformException, SystemExit) as e:
            if isinstance(e, SystemExit):
                self.stop_robot()
                sys.exit(0)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--target_frame', type=str, default='tag_0')
    parser.add_argument('--yaw_tol', type=float, default=0.1)
    parser.add_argument('--target', type=float, default=1.7)
    
    ros_args = rclpy.utilities.remove_ros_args(args=sys.argv)[1:]
    args = parser.parse_args(ros_args)

    rclpy.init()
    node = PrecisionTargetRotateNode(
        target_frame=args.target_frame,
        yaw_tol=args.yaw_tol,
        target_offset=args.target
    )
    
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
