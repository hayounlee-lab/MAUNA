import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from interbotix_xs_msgs.msg import JointGroupCommand
from interbotix_xs_msgs.srv import TorqueEnable, RegisterValues
import argparse
import numpy as np
import threading
import time
import sys

class ArmCommander(Node):
    def __init__(self, arm_name):
        super().__init__('arm_commander')
        self.arm_name = arm_name
        self.curr_joint_positions = None
        self.DT = 0.02  # 50Hz
        self.ERROR_THRESHOLD = 0.1

        # Service clients
        self.torque_client = self.create_client(TorqueEnable, f'/follower_{arm_name}/torque_enable')
        self.reg_client = self.create_client(RegisterValues, f'/follower_{arm_name}/set_motor_registers')
        
        # Topic subscriber and publisher
        self.joint_sub = self.create_subscription(
            JointState, 
            f'/follower_{arm_name}/joint_states', 
            self.joint_state_callback, 
            10
        )
        self.arm_pub = self.create_publisher(
            JointGroupCommand, 
            f'/cmd/{arm_name}_arm', 
            10
        )

    def joint_state_callback(self, msg):
        target_names = ['waist', 'shoulder', 'elbow', 'forearm_roll', 'wrist_angle', 'wrist_rotate']
        try:
            positions = []
            for t_name in target_names:
                idx = next(i for i, m_name in enumerate(msg.name) if t_name in m_name)
                positions.append(msg.position[idx])
            if len(positions) == 6:
                self.curr_joint_positions = positions
        except Exception:
            # Fallback to slicing the first 6 joints if named matching fails
            if msg.position and len(msg.position) >= 6:
                self.curr_joint_positions = list(msg.position[:6])

    def publish_command(self, positions):
        """Common method to publish joint commands"""
        msg = JointGroupCommand(name="arm", cmd=list(positions))
        self.arm_pub.publish(msg)

    def optimize_hardware(self):
        """Initialize hardware profiles to maximize responsiveness"""
        if not self.reg_client.wait_for_service(timeout_sec=1.0):
            return
        for reg_name in ['Profile_Acceleration', 'Profile_Velocity']:
            req = RegisterValues.Request(cmd_type="group", name="arm", reg=reg_name, value=0)
            self.reg_client.call_async(req)

    def secure_torque_on(self):
        """Sequence: Fetch current position -> Enable torque & pre-claim initial pose"""
        self.get_logger().info("Waiting for joint state updates...")
        while rclpy.ok() and self.curr_joint_positions is None:
            time.sleep(0.01)
        
        initial_pose = list(self.curr_joint_positions)
        self.optimize_hardware()

        # Pre-claim current position command before enabling torque
        msg = JointGroupCommand(name="arm", cmd=initial_pose)
        for _ in range(5):
            self.arm_pub.publish(msg)
            time.sleep(0.01)

        if self.torque_client.wait_for_service(timeout_sec=1.0):
            req = TorqueEnable.Request(cmd_type="group", name="arm", enable=True)
            self.torque_client.call_async(req)
        
        time.sleep(0.5)
        return initial_pose

    def check_safety(self, start_pose):
        """Verify the error between physical position and the provided start pose"""
        self.get_logger().info("Checking safety and connection...")
        while rclpy.ok() and self.curr_joint_positions is None:
            self.get_logger().info("Waiting for JointState...", once=True)
            time.sleep(0.1)

        curr = np.array(self.curr_joint_positions)
        start = np.array(start_pose)
        diff = np.linalg.norm(curr - start)
        
        if diff > self.ERROR_THRESHOLD:
            self.get_logger().error(f"Safety Error! Large deviation between physical and start pose. (Diff: {diff:.4f})")
            return False
        
        self.get_logger().info(f"Safety check passed (Diff: {diff:.4f})")
        return True

    def execute_smooth_trajectory(self, p1, p2, duration, label):
        """Smooth movement using Cosine-based S-Curve interpolation"""
        p1 = np.array(p1)
        p2 = np.array(p2)
        num_steps = int(duration / self.DT)
        if num_steps <= 0: 
            num_steps = 1

        self.get_logger().info(f"[{label}] Starting movement for {duration} seconds")
        for t in range(num_steps):
            if not rclpy.ok(): 
                break
            
            # alpha: smoothly transitions from 0 (start) to 1 (end)
            alpha = (1 - np.cos(np.pi * t / num_steps)) / 2
            target_pos = p1 + (p2 - p1) * alpha
            
            self.publish_command(target_pos)
            time.sleep(self.DT)

    def precision_convergence_loop(self, target_np):
        """[Core] Precision error convergence loop using P-Control"""
        self.get_logger().info("Entering precision error convergence loop...")
        convergence_timeout = time.time() + 3.0
        
        while time.time() < convergence_timeout:
            if not rclpy.ok(): 
                break
            
            if self.curr_joint_positions is None:
                time.sleep(0.01)
                continue

            actual_np = np.array(self.curr_joint_positions)
            error = target_np - actual_np
            
            # Terminate if the error of all joints is within 0.003 rad (~0.17 degrees)
            if np.max(np.abs(error)) < 0.003:
                self.get_logger().info("Precision seating successful.")
                break
            
            # Apply P-Control: nudge motor with 20% of the remaining error
            correction_cmd = target_np + (error * 0.2)
            self.publish_command(correction_cmd)
            time.sleep(0.05)

        # Print final results report
        final_actual = np.array(self.curr_joint_positions) if self.curr_joint_positions else target_np
        final_error = target_np - final_actual
        
        print("\n" + "="*60)
        print(" [Final Precision Report]")
        print(f" - Target Pose  : {np.round(target_np, 5)}")
        print(f" - Actual Pose  : {np.round(final_actual, 5)}")
        print(f" - Final Error  : {np.round(final_error, 5)}")
        print(f" - Error Norm   : {np.linalg.norm(final_error):.6f}")
        print("="*60 + "\n")

    def run_pos_mode(self, target_pose, moving_time):
        """Mode A: Check current position -> Torque enable -> S-Curve move -> Precision convergence loop"""
        start_pose_list = self.secure_torque_on()
        if start_pose_list is None: 
            return

        start_pose = np.array(start_pose_list)
        target_np = np.array(target_pose)

        # 1. Main interpolation movement
        self.get_logger().info("Executing interpolated movement...")
        num_steps = int(moving_time / self.DT)
        if num_steps <= 0: num_steps = 1

        for t in range(num_steps):
            if not rclpy.ok(): break
            alpha = (1 - np.cos(np.pi * t / (num_steps - 1))) / 2 if num_steps > 1 else 1.0
            current_step_pose = start_pose + (target_np - start_pose) * alpha
            self.publish_command(current_step_pose)
            time.sleep(self.DT)

        # 2. Precision convergence loop
        self.precision_convergence_loop(target_np)

    def run_sequence_mode(self, start, way, target, total_time):
        """Mode B: Safety check -> Hold start position for stability -> Section-wise S-Curve trajectory"""
        # 1. Safety check
        if not self.check_safety(start):
            sys.exit(1)

        # Optimize hardware profile and enable torque
        self.optimize_hardware()
        if self.torque_client.wait_for_service(timeout_sec=1.0):
            req = TorqueEnable.Request(cmd_type="group", name="arm", enable=True)
            self.torque_client.call_async(req)

        # Prevent drop: Hold start position for 0.5 seconds to secure stable torque
        hold_duration = 0.5
        hold_steps = int(hold_duration / self.DT)
        self.get_logger().info("[HOLD] Holding start position to secure stable torque...")
        for _ in range(hold_steps):
            self.publish_command(start)
            time.sleep(self.DT)

        if way is not None:
            half_time = total_time / 2.0
            self.execute_smooth_trajectory(start, way, half_time, "SECTION 1")
            self.execute_smooth_trajectory(way, target, half_time, "SECTION 2")
        else:
            self.execute_smooth_trajectory(start, target, total_time, "DIRECT")

        # Fix final target position and apply precision convergence loop
        self.publish_command(target)
        time.sleep(0.2)
        self.precision_convergence_loop(np.array(target))
        self.get_logger().info("All movements completed successfully.")


def main():
    parser = argparse.ArgumentParser(description="Interbotix Arm Unified Commander")
    parser.add_argument('--arm', type=str, required=True, help="Arm name (e.g., follower_left)")
    parser.add_argument('--time', type=float, default=4.0, help="Total moving time")
    
    # Mode A: --pos (Compatible with script 1)
    parser.add_argument('--pos', type=float, nargs=6, default=None, help="Target position (6 joints)")
    
    # Mode B: --start, --way, --target (Compatible with script 2)
    parser.add_argument('--start', type=float, nargs=6, default=None, help="Start position (6 joints)")
    parser.add_argument('--way', type=float, nargs=6, default=None, help="Way/Waypoint position (6 joints, optional)")
    parser.add_argument('--target', type=float, nargs=6, default=None, help="Target position (6 joints)")

    args = parser.parse_args()

    rclpy.init()
    commander = ArmCommander(args.arm)
    executor = MultiThreadedExecutor()
    executor.add_node(commander)

    # Background thread to spin the executor in a multi-threaded environment
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        if args.pos is not None:
            # Run Mode A (Script 1 style)
            commander.run_pos_mode(args.pos, args.time)
        elif args.start is not None and args.target is not None:
            # Run Mode B (Script 2 style)
            commander.run_sequence_mode(args.start, args.way, args.target, args.time)
        else:
            commander.get_logger().error("Insufficient arguments. Please specify either '--pos' or '--start' and '--target'.")
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            commander.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
