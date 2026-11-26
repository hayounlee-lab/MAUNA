import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import subprocess
import os

class SequentialExecutor(Node):
    def __init__(self):
        super().__init__('sequential_executor')
        self.target_dir = "/home/aloha/mauna_operation/workflow"
        
        # [Important] Flag to check whether a task is currently running
        self.is_running = False 

        # [Important] QoS settings to prevent message loss (Transient Local)
        # Ensures that late-joining subscribers receive the latest published message.
        qos_profile = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL # Retain message even if connected late
        )

        self.subscription = self.create_subscription(
            String, 'remote_cmd', self.listener_callback, qos_profile)
        
        # Publisher to broadcast execution status and dropped queue info
        self.status_publisher = self.create_publisher(
            String, 'execution_status', qos_profile)

    def listener_callback(self, msg):
        # If another script bundle is already running, strictly ignore new commands.
        if self.is_running:
            self.get_logger().warn("A task is already running. Current request is ignored.")
            return

        try:
            # Set lock by changing state to 'running'.
            self.is_running = True
            
            script_list = [s.strip() for s in msg.data.split('->')]
            self.get_logger().info(f"Task started: {script_list}")

            error_occurred = False

            for i, script in enumerate(script_list):
                full_path = os.path.join(self.target_dir, f"{script}.bash")
                
                if os.path.exists(full_path):
                    self.get_logger().info(f"Executing: {script}.bash")
                    
                    result = subprocess.run(
                        ['bash', full_path],
                        cwd=self.target_dir,
                        capture_output=True,
                        text=True
                    )
                    
                    if result.stdout:
                        self.get_logger().info(f"[{script} output]:\n{result.stdout}")
                    if result.stderr:
                        self.get_logger().error(f"[{script} error]:\n{result.stderr}")
                    
                    # Treat as an error if exit code is not 0
                    if result.returncode != 0:
                        self.get_logger().error(f"Error occurred in {script} (Exit code: {result.returncode})")
                        error_occurred = True
                        
                        # Drop/delete all remaining items in the queue
                        remaining_queue = script_list[i+1:]
                        self.get_logger().warn(f"Discarding remaining queue items: {remaining_queue}")
                        
                        # Publish error status and dropped queue info
                        status_msg = String()
                        status_msg.data = f"ERROR: Failed at '{script}' (Exit code: {result.returncode}). Remaining queue dropped: {remaining_queue}"
                        self.status_publisher.publish(status_msg)
                        break
                else:
                    self.get_logger().error(f"File not found: {full_path}")
                    error_occurred = True
                    
                    # Drop/delete all remaining items in the queue
                    remaining_queue = script_list[i+1:]
                    self.get_logger().warn(f"Discarding remaining queue items: {remaining_queue}")
                    
                    # Publish file not found status and dropped queue info
                    status_msg = String()
                    status_msg.data = f"ERROR: File not found '{script}.bash'. Remaining queue dropped: {remaining_queue}"
                    self.status_publisher.publish(status_msg)
                    break
            
            # If all tasks completed successfully without any error
            if not error_occurred:
                status_msg = String()
                status_msg.data = f"SUCCESS: All tasks completed successfully: {script_list}"
                self.status_publisher.publish(status_msg)
                
        finally:
            # Release lock whether execution finishes safely or encounters an error.
            self.is_running = False
            self.get_logger().info("All tasks completed or aborted. Ready for the next command.")

def main(args=None):
    rclpy.init(args=args)
    node = SequentialExecutor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
