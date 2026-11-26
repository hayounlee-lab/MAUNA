#!/usr/bin/env python3

import os
import h5py
import numpy as np
import rclpy
import argparse
import sys
from rclpy.node import Node
from interbotix_xs_msgs.msg import JointGroupCommand, JointSingleCommand

def smooth_qpos(data_list, steps=5):
    """
    data_list: Stair-like qpos list with step artifacts (N, 14)
    steps: Number of frames the data is held (here, 5)
    """
    data = np.array(data_list)
    num_frames = data.shape[0]
    num_joints = data.shape[1]

    # Create an empty array to store the results
    smoothed_data = np.zeros_like(data)

    # Extract only the indices where data actually updates, e.g., 0, 5, 10...
    real_update_indices = np.arange(0, num_frames, steps)

    for i in range(len(real_update_indices) - 1):
        start_idx = real_update_indices[i]
        end_idx = real_update_indices[i+1]

        start_val = data[start_idx]
        end_val = data[end_idx]

        # Interpolate between start_val and end_val divided by steps
        for j in range(steps):
            # t is a ratio changing from 0 to 1
            t = j / steps
            smoothed_data[start_idx + j] = (1 - t) * start_val + t * end_val

    # Handle the last segment (maintain the last value as there is no next data)
    last_idx = real_update_indices[-1]
    smoothed_data[last_idx:] = data[last_idx]

    return smoothed_data

class AlohaReplayNode(Node):
    def __init__(self, actions):
        super().__init__('aloha_replay_node')
        
        # 1. Setup publishers (reflecting the requested topic format)
        #self.pub_left_arm = self.create_publisher(JointGroupCommand, '/follower_left/commands/joint_group', 10)
        #self.pub_right_arm = self.create_publisher(JointGroupCommand, '/follower_right/commands/joint_group', 10)
        #self.pub_left_gripper = self.create_publisher(JointSingleCommand, '/follower_left/commands/joint_single', 10)
        #self.pub_right_gripper = self.create_publisher(JointSingleCommand, '/follower_right/commands/joint_single', 10)
        self.pub_left_arm = self.create_publisher(JointGroupCommand, '/cmd/left_arm', 10)
        self.pub_right_arm = self.create_publisher(JointGroupCommand, '/cmd/right_arm', 10)
        self.pub_left_gripper = self.create_publisher(JointSingleCommand, '/cmd/left_gripper', 10)
        self.pub_right_gripper = self.create_publisher(JointSingleCommand, '/cmd/right_gripper', 10)
        
        self.actions = actions
        self.step_idx = 0
        self.timer_period = 0.02  # 50Hz (1/50 = 0.02s)
        
        # Reuse message objects to reduce Garbage Collection overhead
        self.arm_msg = JointGroupCommand(name="arm")
        self.gripper_msg = JointSingleCommand(name="gripper")

        print(f"Replay started: Executing a total of {len(self.actions)} steps...")
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        if self.step_idx >= len(self.actions):
            print("Replay has finished.")
            #self.destroy_timer(self.timer)
            self.timer.cancel()
            sys.exit(0)
            #return

        # Extract current step data
        # ALOHA action structure: [L_arm(6), L_grip(1), R_arm(6), R_grip(1)]
        action = self.actions[self.step_idx]
        print(action)

        # --- Left Arm & Gripper ---
        self.arm_msg.cmd = action[0:6].tolist()
        self.pub_left_arm.publish(self.arm_msg)
        
        self.gripper_msg.cmd = float(action[6])
        self.pub_left_gripper.publish(self.gripper_msg)

        # --- Right Arm & Gripper ---
        self.arm_msg.cmd = action[7:13].tolist()
        self.pub_right_arm.publish(self.arm_msg)
        
        self.gripper_msg.cmd = float(action[13])
        self.pub_right_gripper.publish(self.gripper_msg)

        self.step_idx += 1

def load_and_clean_data(file_path):
    """Read the HDF5 file and remove initial Null (0.0) data."""
    with h5py.File(file_path, 'r') as root:
        actions = root['/action'][()]
    
    # Search and remove rows where all joint values are exactly 0.0 (Null data)
    # Use np.any(...) to keep rows where at least one value is non-zero.
    mask = np.any(actions != 0.0, axis=1)
    cleaned_actions = actions[mask]
    cleaned_actions = smooth_qpos(cleaned_actions)
    
    removed_count = len(actions) - len(cleaned_actions)
    if removed_count > 0:
        print(f"Data cleaning complete: Removed {removed_count} initial Null data entries.")
    
    return cleaned_actions

def main(args=None):
    dataset_dir = args['dataset_dir']
    episode_idx = args['episode_idx']
    dataset_name = f'episode_{episode_idx}'
    # Setup file path (modify according to the actual environment)
    dataset_path = os.path.join(dataset_dir, f'episode_{episode_idx}.hdf5')

    if not os.path.exists(dataset_path):
        print(f"Error: File not found -> {dataset_path}")
        return

    # 1. Load and clean data
    cleaned_actions = load_and_clean_data(dataset_path)

    # 2. Run ROS 2
    rclpy.init()
    node = AlohaReplayNode(cleaned_actions)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        #node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--dataset_dir',
        action='store',
        type=str,
        help='Dataset dir.',
        # default=DATA_DIR,
        required=True,
    )
    parser.add_argument(
        '--episode_idx',
        action='store',
        type=int,
        help='Episode index.',
        default=0,
        required=False,
    )
    main(vars(parser.parse_args()))
