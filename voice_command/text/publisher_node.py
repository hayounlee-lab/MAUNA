import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

rclpy.init()

class CommandPublisher(Node):
    def __init__(self):
        super().__init__('command_publisher') 

        qos_profile = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.publisher_ = self.create_publisher(
            String, 
            'remote_cmd', 
            qos_profile
        )
        self.get_logger().info("Command Publisher initialized with Transient Local QoS")

    def publish(self, text):
        msg = String()
        msg.data = text
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published message: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = CommandPublisher()

    node.send_command('a->b->c')

    import time
    time.sleep(0.5) 

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
