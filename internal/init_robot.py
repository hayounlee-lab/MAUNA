#!/usr/bin/env python3

import rclpy
from aloha.constants import (
    FOLLOWER_GRIPPER_JOINT_CLOSE,
    LEADER_GRIPPER_JOINT_MID,
    START_ARM_POSE,
)
from aloha.robot_utils import (
    move_arms,
    move_grippers,
    torque_on,
)
from interbotix_common_modules.common_robot.robot import (
    create_interbotix_global_node,
    robot_shutdown,
    robot_startup,
)
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

def opening_ceremony(
    follower_bot_left: InterbotixManipulatorXS,
    follower_bot_right: InterbotixManipulatorXS
) -> None:
    """Configure robot hardware modes and move to initial positions"""
    
    # 1. Reboot motors and set operating modes (Arms: position control, Grippers: current-based position control)
    for bot in [follower_bot_left, follower_bot_right]:
        bot.core.robot_reboot_motors('single', 'gripper', True)
        bot.core.robot_set_operating_modes('group', 'arm', 'position')
        bot.core.robot_set_operating_modes('single', 'gripper', 'current_based_position')
        bot.core.robot_set_motor_registers('single', 'gripper', 'current_limit', 300)

    # 2. Turn on torque (apply motor power)
    torque_on(follower_bot_left)
    torque_on(follower_bot_right)

    # 3. Move arms to the starting pose
    print("Moving arms to starting position...")
    start_arm_qpos = START_ARM_POSE[:6]
    move_arms(
        [follower_bot_left, follower_bot_right],
        [start_arm_qpos] * 2,
        moving_time=4.0,
    )

    # 4. Move grippers to the starting position
    print("Moving grippers to starting position...")
    move_grippers(
        [follower_bot_left, follower_bot_right],
        [LEADER_GRIPPER_JOINT_MID, FOLLOWER_GRIPPER_JOINT_CLOSE] * 2,
        moving_time=0.5
    )

def main() -> None:
    node = create_interbotix_global_node('aloha_init')
    
    # Initialize robot objects
    follower_bot_left = InterbotixManipulatorXS(robot_model='vx300s', robot_name='follower_left', node=node)
    follower_bot_right = InterbotixManipulatorXS(robot_model='vx300s', robot_name='follower_right', node=node)
    #leader_bot_left = InterbotixManipulatorXS(robot_model='wx250s', robot_name='leader_left', node=node)
    #leader_bot_right = InterbotixManipulatorXS(robot_model='wx250s', robot_name='leader_right', node=node)

    # ROS 2 startup
    robot_startup(node)

    # Perform initialization and opening ceremony
    print("Starting initialization...")
    opening_ceremony(
        follower_bot_left,
        follower_bot_right,
    )

    print("Initialization complete. All robots are ready.")
    
    # Shutdown node after initialization (torque remains enabled)
    robot_shutdown(node)

if __name__ == '__main__':
    main()
