export INTERBOTIX_ALOHA_IS_MOBILE=true # true for Mobile, false for Stationary
source ~/interbotix_ws/install/setup.bash # configure ROS workspace environment
python3 -c "
import rclpy
from std_srvs.srv import SetBool
rclpy.init()
node = rclpy.create_node('temp_client')
client = node.create_client(SetBool, '/mobile_base/set_motor_torque_status')
if client.wait_for_service(timeout_sec=2.0):
    req = SetBool.Request(data=True)
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
    if future.done():
        print('\n🎯 [Success] Server response:', future.result().message)
    else:
        print('\n⚠️ [Timeout] The service request was sent, but the motor node is not responding.')
else:
    print('\n❌ [Error] The /mobile_base node is turned off or the service cannot be found.')
    exit(1)
node.destroy_node()
rclpy.shutdown()
"
