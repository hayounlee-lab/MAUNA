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

class PrecisionTargetMoveNode(Node):
    def __init__(self, target_frame='tag_1', x_tol=0.008, yaw_tol=0.5, target_dist=0.875, target_offset=1.9):
        super().__init__('precision_target_move_node')
        
        # --- [S30 Optimized Configuration Values] ---
        self.target_frame = target_frame
        self.source_frame = 'camera' 
        self.TARGET_DIST_X = target_dist + 0.39
        self.target_offset = target_offset 
        
        self.X_TOL = x_tol                  
        self.YAW_TOL = math.radians(yaw_tol)

        # Velocities reflecting S30 hardware characteristics
        self.MAX_LIN_VEL = 0.25        # Lowered max linear velocity for approach (0.5 is too fast)
        self.MIN_LIN_VEL = 0.12        # Minimum velocity required to start S30 motor movement (Important)
        self.MAX_ANG_VEL = 0.20        # Maximum angular velocity
        self.MIN_ANG_VEL = 0.10        # Minimum angular velocity
        
        self.KP_LIN = 1.2              # Proportional control constant for distance
        self.KP_ANG = 1.0              # Proportional control constant for angle

        # --- [State Management] ---
        self.AVG_SIZE = 15             # Increased moving average filter size to account for S30 shaking/vibration
        self.dist_history = []
        self.yaw_history = []
        
        # States: MEASURE (Stop/Measure), MOVE (Smooth movement), STOP_WAIT (Inertia waiting)
        self.state = "MEASURE" 
        self.move_start_time = None
        
        self.current_avg_dist = 0.0
        self.current_avg_yaw = 0.0
        self.is_tag_visible = False

        self.publisher = self.create_publisher(Twist, '/mobile_base/cmd_vel', 10)
        self.detection_sub = self.create_subscription(
            AprilTagDetectionArray, '/detections', self.detection_callback, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Control loop frequency 20Hz (for S30 communication stability)
        self.timer = self.create_timer(1.0/20.0, self.control_loop)
        self.get_logger().info(f"🚀 S30 precision approach started: Target={target_dist}m, X_Tol={x_tol}m")

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
            t = self.tf_buffer.lookup_transform(self.source_frame, self.target_frame, rclpy.time.Time())
            
            # Calculate Yaw
            q = t.transform.rotation
            _, yaw_rad, _ = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
            yaw_deg = math.degrees(yaw_rad) - self.target_offset
            if yaw_deg > 90: yaw_deg -= 180
            elif yaw_deg < -90: yaw_deg += 180
            current_yaw_rad = math.radians(yaw_deg)

            # Calculate distance
            current_dist = t.transform.translation.z + 0.39

            # --- [S30-Specific State Machine] ---
            
            if self.state == "MEASURE":
                self.stop_robot()
                self.dist_history.append(current_dist)
                self.yaw_history.append(current_yaw_rad)

                if len(self.dist_history) >= self.AVG_SIZE:
                    self.current_avg_dist = sum(self.dist_history) / len(self.dist_history)
                    self.current_avg_yaw = sum(self.yaw_history) / len(self.yaw_history)
                    
                    self.dist_history = []
                    self.yaw_history = []

                    x_error = self.current_avg_dist - self.TARGET_DIST_X
                    
                    if abs(x_error) <= self.X_TOL and abs(self.current_avg_yaw) <= self.YAW_TOL:
                        self.get_logger().info(f"📊 Average Distance deviation: {x_error:.5f}m")
                        self.get_logger().info("🎯 [Success] Target destination settlement completed!")
                        raise SystemExit
                    
                    self.state = "MOVE"
                    self.move_start_time = now_time

            elif self.state == "MOVE":
                twist = Twist()
                x_error = self.current_avg_dist - self.TARGET_DIST_X
                
                # 1. Determine movement duration (longer for larger errors, shorter "bump" for smaller ones)
                # S30 requires at least 0.2 seconds to actually move.
                move_duration = 0.5 if abs(x_error) > 0.05 else 0.2
                elapsed = (now_time - self.move_start_time).nanoseconds / 1e9

                if elapsed < move_duration:
                    # Prioritize angle adjustment
                    if abs(self.current_avg_yaw) > self.YAW_TOL:
                        target_ang = self.current_avg_yaw * -self.KP_ANG
                        twist.angular.z = math.copysign(
                            max(self.MIN_ANG_VEL, min(abs(target_ang), self.MAX_ANG_VEL)), target_ang)
                    # Distance adjustment
                    elif abs(x_error) > self.X_TOL:
                        target_lin = x_error * self.KP_LIN
                        twist.linear.x = math.copysign(
                            max(self.MIN_LIN_VEL, min(abs(target_lin), self.MAX_LIN_VEL)), x_error)
                    
                    self.publisher.publish(twist)
                else:
                    if abs(self.current_avg_yaw) < self.YAW_TOL:
                      self.get_logger().info(f"📊 Average Distance deviation: {x_error:.5f}m")
                    self.stop_robot()
                    self.state = "STOP_WAIT"
                    self.move_start_time = now_time # Reused for measuring wait time

            elif self.state == "STOP_WAIT":
                # Wait for the heavy body of S30 to come to a complete stop (1.5 seconds)
                elapsed = (now_time - self.move_start_time).nanoseconds / 1e9
                if elapsed > 1.5:
                    self.state = "MEASURE"

        except (TransformException, SystemExit) as e:
            if isinstance(e, SystemExit):
                self.stop_robot()
                sys.exit(0)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--target_frame', type=str, default='tag_0')
    parser.add_argument('--x_tol', type=float, default=0.003)  # 8mm
    parser.add_argument('--yaw_tol', type=float, default=0.03)  # 0.5 degrees
    parser.add_argument('--dist', type=float, default=1.61)   # 0.875m
    parser.add_argument('--target', type=float, default=1.7)   # 1.9 degrees

    ros_args = rclpy.utilities.remove_ros_args(args=sys.argv)[1:]
    args = parser.parse_args(ros_args)

    rclpy.init()
    node = PrecisionTargetMoveNode(
        target_frame=args.target_frame,
        x_tol=args.x_tol,
        yaw_tol=args.yaw_tol,
        target_dist=args.dist,
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
