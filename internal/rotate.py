import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import tf_transformations
import argparse
import sys

class RotateNode(Node):
    def __init__(self, target_deg=-90.0):
        super().__init__('robust_rotate_node')
        self.publisher = self.create_publisher(Twist, '/mobile_base/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Odometry, '/mobile_base/odom', self.odom_callback, 10
        )
        
        self.current_pose = None
        self.last_valid_pose = None
        self.initial_yaw = None
        self.last_msg_time = self.get_clock().now()

        # --- Configuration (Robustness Safety Parameters) ---
        self.max_allowed_jump = 0.15  # Max allowed position jump for noise filtering (m)
        self.vo_timeout = 0.5         # Odometry communication timeout (seconds)
        
        # Target angle configuration (converted from degrees to radians)
        self.target_angle = math.radians(target_deg)
        
        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg):
        now = self.get_clock().now()
        new_pose = msg.pose.pose

        # 1. Sensor data jump filtering (Noise prevention)
        if self.last_valid_pose is not None:
            dist_change = self.get_distance_2d(self.last_valid_pose, new_pose)
            if dist_change > self.max_allowed_jump:
                self.get_logger().warn(f"Anomaly detected (Ignored): {dist_change:.2f}m jump")
                return 

        self.current_pose = new_pose
        self.last_valid_pose = new_pose
        self.last_msg_time = now

    def control_loop(self):
        # 2. Communication timeout check
        time_since_last_msg = (self.get_clock().now() - self.last_msg_time).nanoseconds / 1e9
        if time_since_last_msg > self.vo_timeout:
            self.get_logger().error("Odometry lost! Stopping for safety.")
            self.stop_robot()
            return

        if self.current_pose is None:
            return

        # 3. Set the initial reference yaw on first valid pose
        if self.initial_yaw is None:
            self.initial_yaw = self.get_yaw(self.current_pose)
            self.get_logger().info(f"Initial pose set. Starting rotation to target: {math.degrees(self.target_angle):.1f} deg")
            return

        self.execute_rotation()

    def execute_rotation(self):
        twist = Twist()
        current_yaw = self.get_yaw(self.current_pose)
        
        # Calculate current accumulated rotation angle (considering rotation direction)
        angle_diff = self.normalize_angle(current_yaw - self.initial_yaw)
        
        # Calculate remaining angle (absolute value basis)
        remaining_angle = abs(self.target_angle) - abs(angle_diff)
        
        if remaining_angle <= 0.03:  # Tolerance reached
            self.stop_robot()
            self.get_logger().info("Rotation mission completed!")
            self.timer.cancel()
            raise SystemExit 
        else:
            # Set direction: negative target means clockwise(-), positive means counter-clockwise(+)
            direction = 1.0 if self.target_angle > 0 else -1.0
            # Calculate speed (ensure minimum speed 0.1, max capped at 0.4)
            speed = min(0.4, remaining_angle * 1.5 + 0.1)
            twist.angular.z = direction * speed

        self.publisher.publish(twist)

    def stop_robot(self):
        self.publisher.publish(Twist())

    def get_distance_2d(self, p1, p2):
        # Calculate 2D Euclidean distance for jump filtering
        return math.sqrt((p1.position.x - p2.position.x)**2 + (p1.position.y - p2.position.y)**2)

    def get_yaw(self, pose):
        q = pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        return yaw

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--target', type=float, default=-90, help='Target rotation angle in degrees')

    ros_args = rclpy.utilities.remove_ros_args(args=sys.argv)[1:]
    args = parser.parse_args(ros_args)

    rclpy.init()
    node = RotateNode(target_deg=args.target)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        node.get_logger().info("Shutting down...")
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
