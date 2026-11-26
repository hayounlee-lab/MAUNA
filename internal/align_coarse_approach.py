import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from apriltag_msgs.msg import AprilTagDetectionArray
from tf2_ros import Buffer, TransformListener, TransformException
import math
import tf_transformations
import argparse
import sys

class PrecisionTagFollower(Node):
    def __init__(self, target_frame='tag_0', yaw_tol=0.05, target_dist=1.61, target_offset=1.7):
        super().__init__('precision_tag_follower')
        
        # --- [Configuration Values] ---
        self.TARGET_ID = int(target_frame.split('_')[-1])
        self.target_offset = target_offset
        self.OFFSET = 0.39
        self.TARGET_X = target_dist + self.OFFSET  
        
        self.YAW_TOL = math.radians(yaw_tol)
        self.MIN_ANG_VEL = 0.14
        self.MAX_ANG_VEL = 0.3
        
        # Newly added configurations
        self.INITIAL_APPROACH_OFFSET = math.radians(15.0) # 15-degree offset at the start
        self.START_DISTANCE = None # Stores the distance when the tag is first detected

        # --- [State and Memory Variables] ---
        self.state = 'SEARCHING'
        self.is_tag_visible = False
        self.last_dy = 0.0

        self.publisher = self.create_publisher(Twist, '/mobile_base/cmd_vel', 10)
        self.detection_sub = self.create_subscription(
            AprilTagDetectionArray, '/detections', self.detection_callback, 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(1.0/20.0, self.control_loop)
        self.get_logger().info("🚀 Dynamic offset follower started (Applied initial deviation)")

    def detection_callback(self, msg):
        self.is_tag_visible = any(d.id == self.TARGET_ID for d in msg.detections)

    def get_tag_data(self):
        if not self.is_tag_visible:
            return None, None, None

        try:
            t = self.tf_buffer.lookup_transform(
                'camera', 
                f'tag_{self.TARGET_ID}', 
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05))
            
            cam_z = t.transform.translation.z
            cam_x = t.transform.translation.x
            q = t.transform.rotation
            _, raw_yaw, _ = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
            
            yaw_deg = math.degrees(raw_yaw) + self.target_offset
            if yaw_deg > 90: yaw_deg -= 180
            elif yaw_deg < -90: yaw_deg += 180
            yaw_rad = math.radians(yaw_deg)

            dist_x = cam_z + (math.cos(raw_yaw) * self.OFFSET)
            dist_y = (-cam_x) + (math.sin(raw_yaw) * self.OFFSET)
            
            self.last_dy = dist_y
            return dist_x, dist_y, yaw_rad
        except TransformException:
            return None, None, None

    def ensure_min_velocity(self, val, min_val):
        if abs(val) < 0.001: return 0.0
        return math.copysign(max(abs(val), min_val), val)

    def control_loop(self):
        dx, dy, dyaw = self.get_tag_data()
        twist = Twist()

        # --- [Recovery behavior when the tag is lost] ---
        if dx is None:
            if self.state in ['APPROACH', 'FINAL_ALIGN']:
                self.get_logger().warn("⚠️ Tag lost! Recovering line of sight...")
                direction = -1.0 if self.last_dy > 0 else 1.0
                twist.angular.z = direction * self.MIN_ANG_VEL
                self.publisher.publish(twist)
            else:
                self.stop_robot()
            return

        # --- [State Machine] ---
        if self.state == 'SEARCHING':
            if 0.5 < dx < 10.0: # Expanded detection range
                self.START_DISTANCE = dx # Record the starting distance
                self.state = 'APPROACH'

        elif self.state == 'APPROACH':
            if dx <= self.TARGET_X:
                self.state = 'FINAL_ALIGN'
                return

            # 1. Calculate linear velocity
            dist_error = dx - self.TARGET_X
            twist.linear.x = min(0.2, dist_error * 0.4 + 0.05)

            # 2. Calculate dynamic offset (converges from 15 degrees to 0 degrees as it gets closer)
            # Calculate remaining distance ratio (0.0 ~ 1.0)
            remaining_ratio = max(0.0, min(1.0, (dx - self.TARGET_X) / (self.START_DISTANCE - self.TARGET_X + 0.1)))
            current_dynamic_offset = self.INITIAL_APPROACH_OFFSET * remaining_ratio
            
            # Add offset to the direction where the tag is located (target_angle)
            # Determine the offset direction based on the sign of dy to guide faster turning toward the center
            side_sign = 1.0 if dy > 0 else -1.0
            target_angle = math.atan2(dy, dx) + (side_sign * current_dynamic_offset)

            # 3. Control angular velocity
            raw_ang = target_angle * 0.5 # Adjust gain
            twist.angular.z = self.ensure_min_velocity(raw_ang, self.MIN_ANG_VEL)
            twist.angular.z = max(-self.MAX_ANG_VEL, min(self.MAX_ANG_VEL, twist.angular.z))

        elif self.state == 'FINAL_ALIGN':
            # Final alignment: Face the exact center of the tag without any offset
            target_angle = math.atan2(dy, dx)
            if abs(target_angle) < self.YAW_TOL:
                self.get_logger().info("🎯 Reached target position and alignment completed")
                self.stop_robot()
                raise SystemExit
            else:
                raw_ang = target_angle * 2.0
                twist.angular.z = self.ensure_min_velocity(raw_ang, self.MIN_ANG_VEL)
                twist.angular.z = max(-0.2, min(0.2, twist.angular.z))

        self.publisher.publish(twist)

    def stop_robot(self):
        self.publisher.publish(Twist())

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--target_frame', type=str, default='tag_0')
    parser.add_argument('--yaw_tol', type=float, default=3.00)
    parser.add_argument('--target', type=float, default=1.7)
    parser.add_argument('--dist', type=float, default=1.61)

    args = parser.parse_args(rclpy.utilities.remove_ros_args(args=sys.argv)[1:])
    rclpy.init()
    node = PrecisionTagFollower(args.target_frame, args.yaw_tol, args.dist, args.target)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
