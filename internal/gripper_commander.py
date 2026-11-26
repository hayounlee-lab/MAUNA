import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from interbotix_xs_msgs.msg import JointSingleCommand  # Use Single Command for gripper
import argparse
import numpy as np
import time

class GripperCommander(Node):
    def __init__(self, arm_name):
        super().__init__('gripper_commander')
        self.arm_name = arm_name
        self.curr_gripper_position = None
        self.DT = 0.02  # Control period (50Hz)

        # Subscribe to current state (to track gripper position)
        self.joint_sub = self.create_subscription(
            JointState,
            f'/follower_{arm_name}/joint_states',
            self.joint_state_callback,
            10
        )

        # Gripper command publisher
        self.gripper_pub = self.create_publisher(
            JointSingleCommand,
            f'/cmd/{arm_name}_gripper',
            10
        )

    def joint_state_callback(self, msg):
        """
        Find and store the position of the joint named 'gripper' from the JointState message.
        """
        try:
            # Find index of the gripper joint and store its current position
            idx = msg.name.index('gripper')
            self.curr_gripper_position = msg.position[idx]
        except ValueError:
            # Handle cases where gripper information is missing in the message
            pass

    def move_gripper_smoothly(self, target_pose, moving_time):
        """Implement move_grippers logic as a class method"""
        
        # 1. Wait until the current gripper position is received
        self.get_logger().info("Waiting to receive current gripper position...")
        while rclpy.ok() and self.curr_gripper_position is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        start_pose = self.curr_gripper_position
        
        # 2. Calculate interpolation steps (trajectory generation)
        num_steps = int(moving_time / self.DT)
        if num_steps <= 0: num_steps = 1
        
        # Generate 1D trajectory using np.linspace
        traj = np.linspace(start_pose, target_pose, num_steps)

        self.get_logger().info(f"Starting gripper movement: {num_steps} steps over {moving_time} seconds")

        # 3. Send individual commands in a loop
        gripper_command = JointSingleCommand()
        gripper_command.name = 'gripper'

        for t in range(num_steps):
            gripper_command.cmd = float(traj[t]) # Convert np.float64 to standard float
            self.gripper_pub.publish(gripper_command)
            
            # Wait for the specified control period
            time.sleep(self.DT)

        self.get_logger().info("Gripper movement completed.")

def main():
    parser = argparse.ArgumentParser(description='Interbotix Gripper Smooth Commander')
    parser.add_argument('--arm', type=str, required=True, help='Arm name (e.g., leader_left)')
    # Gripper typically takes a single value, so only one float is needed.
    parser.add_argument('--pos', type=float, required=True, help='Target gripper position (usually between 0.0 and 1.0, depending on robot settings)')
    parser.add_argument('--time', type=float, default=2.0, help='Total movement time (seconds)')

    args = parser.parse_args()

    rclpy.init()
    commander = GripperCommander(args.arm)

    try:
        commander.move_gripper_smoothly(args.pos, args.time)
    except KeyboardInterrupt:
        pass
    finally:
        commander.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
