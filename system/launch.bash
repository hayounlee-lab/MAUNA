export INTERBOTIX_ALOHA_IS_MOBILE=true # true for Mobile, false for Stationary
source ~/interbotix_ws/install/setup.bash # configure ROS workspace environment
cd ~/mauna_operation/launch
ros2 launch ./mauna.launch.py use_base:='true' use_joystick_teleop:='true'
#ros2 launch aloha master.launch.py use_base:='true' use_joystick_teleop:='true'

