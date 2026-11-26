#!/usr/bin/env python3

import argparse
import math
import sys

import numpy as np
import rclpy
from rclpy.node import Node

from apriltag_msgs.msg import AprilTagDetectionArray
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener, TransformException
import tf_transformations


class RobotStateVerifier(Node):
    def __init__(self, args):
        super().__init__('robot_state_verifier')

        self.args = args
        self.mode = args.mode
        self.finished = False
        self.exit_code = 1
        self.start_time = self.get_clock().now()

        if self.mode in ('tag', 'position'):
            self.target_frame = args.target_frame
            self.target_id = int(self.target_frame.split('_')[-1])
            self.discovered_tags = set()
            self.is_tag_visible = False
            self.miss_count = 0

            self.detection_sub = self.create_subscription(
                AprilTagDetectionArray,
                '/detections',
                self.detection_callback,
                10
            )

        if self.mode == 'position':
            self.yaw_history = []
            self.dist_x_history = []
            self.dist_y_history = []

            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

            self.position_timer = self.create_timer(
                1.0 / 30.0,
                self.position_control_loop
            )

        elif self.mode == 'arm':
            self.pose_0 = np.array(args.pose_0, dtype=float)
            self.pose_1 = np.array(args.pose_1, dtype=float)

            topic = f'/follower_{args.arm}/joint_states'

            self.joint_sub = self.create_subscription(
                JointState,
                topic,
                self.joint_state_callback,
                10
            )

            self.get_logger().info(f"Monitoring joint states: {topic}")

        if self.mode == 'tag':
            self.get_logger().info(
                f"Checking for AprilTag: {self.target_frame}"
            )

        elif self.mode == 'position':
            self.get_logger().info(
                f"Checking tag-relative position: {self.target_frame}"
            )

        self.timeout_timer = self.create_timer(
            0.1,
            self.timeout_callback
        )

    def timeout_callback(self):
        if self.finished:
            return

        elapsed = (
            self.get_clock().now() - self.start_time
        ).nanoseconds / 1e9

        if elapsed >= self.args.timeout:
            self.finish(False, "Verification timed out.")

    def detection_callback(self, msg):
        if self.finished:
            return

        for detection in msg.detections:
            self.discovered_tags.add(detection.id)

        self.is_tag_visible = any(
            detection.id == self.target_id
            for detection in msg.detections
        )

        if self.mode == 'tag':
            if self.is_tag_visible:
                self.finish(True, f"Detected {self.target_frame}.")
            return

        if self.is_tag_visible:
            self.miss_count = 0
        else:
            self.miss_count += 1

            if self.miss_count >= self.args.max_miss:
                self.finish(
                    False,
                    f"Could not detect {self.target_frame}."
                )

    @staticmethod
    def cosine_similarity(v1, v2):
        norm1 = np.linalg.norm(v1)
        norm2 = np.linalg.norm(v2)

        if norm1 == 0.0 or norm2 == 0.0:
            return 0.0

        return float(np.dot(v1, v2) / (norm1 * norm2))

    def joint_state_callback(self, msg):
        if self.finished or len(msg.position) < 6:
            return

        current_pose = np.array(msg.position[:6], dtype=float)

        if not np.all(np.isfinite(current_pose)):
            return

        sim_0 = self.cosine_similarity(current_pose, self.pose_0)
        sim_1 = self.cosine_similarity(current_pose, self.pose_1)

        self.get_logger().info(
            f"Pose 0 similarity: {sim_0:.4f}; "
            f"Pose 1 similarity: {sim_1:.4f}",
            throttle_duration_sec=1.0
        )

        if sim_0 >= self.args.joint_threshold:
            self.finish(True, f"Arm matches reference pose 0 ({sim_0:.4f}).", pose_idx=0)
        elif sim_1 >= self.args.joint_threshold:
            self.finish(True, f"Arm matches reference pose 1 ({sim_1:.4f}).", pose_idx=1)

    def position_control_loop(self):
        if self.finished or self.mode != 'position':
            return

        if not self.is_tag_visible:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.args.source_frame,
                self.target_frame,
                rclpy.time.Time()
            )
        except TransformException:
            self.get_logger().warning(
                "Waiting for the AprilTag TF transform.",
                throttle_duration_sec=2.0
            )
            return

        translation = transform.transform.translation
        rotation = transform.transform.rotation

        _, raw_yaw, _ = tf_transformations.euler_from_quaternion([
            rotation.x,
            rotation.y,
            rotation.z,
            rotation.w
        ])

        yaw_deg = math.degrees(raw_yaw) - self.args.target

        if yaw_deg > 90:
            yaw_deg -= 180
        elif yaw_deg < -90:
            yaw_deg += 180

        yaw_rad = math.radians(yaw_deg)

        dist_x = translation.z
        dist_y = (
            -translation.x
            + math.sin(yaw_rad) * self.args.offset
        )

        self.yaw_history.append(yaw_rad)
        self.dist_x_history.append(dist_x)
        self.dist_y_history.append(dist_y)

        if len(self.yaw_history) < self.args.no:
            return

        avg_yaw = math.degrees(
            sum(self.yaw_history) / len(self.yaw_history)
        )
        avg_x = sum(self.dist_x_history) / len(self.dist_x_history)
        avg_y = sum(self.dist_y_history) / len(self.dist_y_history)

        if self.args.within:
            err_x = abs(abs(avg_x) - self.args.dist)
        else:
            err_x = abs(avg_x - self.args.dist)

        err_y = abs(avg_y)
        err_yaw = abs(avg_yaw)

        passed_x = err_x <= self.args.tol_x
        passed_y = err_y <= self.args.tol_y
        passed_yaw = err_yaw <= self.args.tol_yaw

        self.get_logger().info(
            f"Average X: {avg_x:.5f} m; error: {err_x:.5f} m"
        )
        self.get_logger().info(
            f"Average Y: {avg_y:.5f} m; error: {err_y:.5f} m"
        )
        self.get_logger().info(
            f"Average yaw: {avg_yaw:.3f} deg; error: {err_yaw:.3f} deg"
        )

        success = passed_x and passed_y and passed_yaw

        self.finish(
            success,
            "Tag-relative position is within tolerance."
            if success
            else "Tag-relative position is outside tolerance."
        )

    def finish(self, success, reason, pose_idx=None):
        if self.finished:
            return

        self.finished = True
        if success:
            self.exit_code = 2 if pose_idx == 1 else 0
        else:
            self.exit_code = 1

        self.get_logger().info(reason)
        print("SUCCESS" if success else "FAIL", flush=True)
        # Set flag only within callbacks and leave exit handling to the main loop


def parse_args():
    parser = argparse.ArgumentParser(
        description="Verify one selected robot state."
    )

    parser.add_argument(
        '--mode',
        required=True,
        choices=['tag', 'position', 'arm'],
        help='Verification mode: tag, position, or arm.'
    )

    parser.add_argument(
        '--timeout',
        type=float,
        default=30.0,
        help='Maximum verification time in seconds.'
    )

    parser.add_argument('--target_frame', default='tag_0')
    parser.add_argument('--source_frame', default='camera')
    parser.add_argument('--target', type=float, default=1.7)
    parser.add_argument('--dist', type=float, default=1.7)
    parser.add_argument('--no', type=int, default=10)
    parser.add_argument('--tol_x', type=float, default=0.05)
    parser.add_argument('--tol_y', type=float, default=0.05)
    parser.add_argument('--tol_yaw', type=float, default=2.0)
    parser.add_argument('--max_miss', type=int, default=10)
    parser.add_argument('--offset', type=float, default=0.39)
    parser.add_argument('--within', action='store_true')

    parser.add_argument(
        '--arm',
        choices=['right', 'left'],
        default='right'
    )
    parser.add_argument(
        '--pose-0',
        nargs=6,
        type=float,
        default=[-0.5737, 0.1933, 0.7977, -0.0061, -0.2899, -0.0015]
    )
    parser.add_argument(
        '--pose-1',
        nargs=6,
        type=float,
        default=[-0.0813, 0.0092, 0.9020, -0.0046, -0.4403, -0.0245]
    )
    parser.add_argument(
        '--joint-threshold',
        type=float,
        default=0.90
    )

    ros_args = rclpy.utilities.remove_ros_args(args=sys.argv)[1:]
    return parser.parse_args(ros_args)


def main():
    args = parse_args()

    if args.timeout <= 0 or args.no < 1 or args.max_miss < 1:
        print("FAIL", flush=True)
        print(
            "Error: timeout, no, and max_miss must be positive.",
            file=sys.stderr
        )
        return 1

    if not -1.0 <= args.joint_threshold <= 1.0:
        print("FAIL", flush=True)
        print(
            "Error: joint-threshold must be between -1 and 1.",
            file=sys.stderr
        )
        return 1

    rclpy.init()
    node = RobotStateVerifier(args)

    try:
        # Use a spin_once loop instead of rclpy.spin(node) to exit immediately upon success
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        node.exit_code = 130
    finally:
        result_code = node.exit_code

        if rclpy.ok():
            rclpy.shutdown()

        node.destroy_node()

    return result_code


if __name__ == '__main__':
    sys.exit(main())
