export INTERBOTIX_ALOHA_IS_MOBILE=true # true for Mobile, false for Stationary
#source /opt/ros/humble/setup.bash # configure ROS system install environment
source ~/interbotix_ws/install/setup.bash # configure ROS workspace environment
bash move_to_tag.bash w
sleep 5
cd ~/mauna_operation/internal
python3 read_whiteboard.py
