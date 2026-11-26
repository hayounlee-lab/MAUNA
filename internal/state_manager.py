import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from interbotix_xs_msgs.msg import JointGroupCommand, JointSingleCommand 
import numpy as np

class RobotStateManager(Node):
    def __init__(self):
        super().__init__('robot_state_manager')

        self.command_memory = {
            'left_arm': [0.0] * 6,
            'right_arm': [0.0] * 6,
            'left_gripper': 0.0,
            'right_gripper': 0.0
        }

        self.state_pub = self.create_publisher(Float64MultiArray, '/total_command_status', 10)
        self.left_arm_pub = self.create_publisher(JointGroupCommand, '/follower_left/commands/joint_group', 10)
        self.right_arm_pub = self.create_publisher(JointGroupCommand, '/follower_right/commands/joint_group', 10)
        self.left_gripper_pub = self.create_publisher(JointSingleCommand, '/follower_left/commands/joint_single', 10)
        self.right_gripper_pub = self.create_publisher(JointSingleCommand, '/follower_right/commands/joint_single', 10)

        self.create_subscription(JointGroupCommand, '/cmd/left_arm', lambda msg: self.update_arm_command('left', msg.cmd), 10)
        self.create_subscription(JointGroupCommand, '/cmd/right_arm', lambda msg: self.update_arm_command('right', msg.cmd), 10)
        self.create_subscription(JointSingleCommand, '/cmd/left_gripper', lambda msg: self.update_gripper_command('left', msg.cmd), 10)
        self.create_subscription(JointSingleCommand, '/cmd/right_gripper', lambda msg: self.update_gripper_command('right', msg.cmd), 10)

        self.timer = self.create_timer(0.1, self.publish_remembered_state)

    def update_arm_command(self, arm_side, positions):
        """Store the arm command and forward it to the actual robot."""
        self.command_memory[f'{arm_side}_arm'] = list(positions)
        
        msg = JointGroupCommand(name="arm", cmd=list(positions))
        if arm_side == 'left':
            self.left_arm_pub.publish(msg)
        else:
            self.right_arm_pub.publish(msg)

    def update_gripper_command(self, side, position):
        """Store the gripper command and forward it to the actual robot."""
        self.command_memory[f'{side}_gripper'] = position

        msg = JointSingleCommand(name="gripper", cmd=position)
        if side == 'left':
            self.left_gripper_pub.publish(msg)
        else:
            self.right_gripper_pub.publish(msg)

    def publish_remembered_state(self):
        action = np.zeros(14)

        action[:6] = self.command_memory['left_arm']
        action[6] = float(self.command_memory['left_gripper'])
        action[7:13] = self.command_memory['right_arm']
        action[13] = float(self.command_memory['right_gripper'])
        
        msg = Float64MultiArray()
        msg.data = action.tolist()
        self.state_pub.publish(msg)

    def publish_remembered_state_v2(self):
        """Convert the remembered state into a 14-dimensional fixed array format and publish it."""
        
        action = np.zeros(14)

        action[:6] = self.command_memory['left_arm']
        action[6] = float(self.command_memory['left_gripper'])
        action[7:13] = self.command_memory['right_arm']
        action[13] = float(self.command_memory['right_gripper'])

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [f'l_j{i}' for i in range(6)] + ['l_gripper'] + \
                   [f'r_j{i}' for i in range(6)] + ['r_gripper']
        msg.position = action.tolist() 
        
        self.state_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotStateManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
