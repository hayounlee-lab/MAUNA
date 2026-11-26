export INTERBOTIX_ALOHA_IS_MOBILE=true # true for Mobile, false for Stationary
source /opt/ros/humble/setup.bash # configure ROS system install environment
source ~/interbotix_ws/install/setup.bash # configure ROS workspace environment
cd ~/mauna_operation/internal
python3 gripper_commander.py --arm right --pos 1.5923 --time 1.0
python3 gripper_commander.py --arm right --pos 0.0185 --time 1.0
python3 arm_commander.py --arm right --time 4.0 --pos 0.8  0.93  -0.38 0.0  0.85  0.0 #new home
python3 arm_commander.py --arm right --time 2.5 --start 0.8  0.93  -0.38 0.0  0.85  0.0 --target 0.8  0.88  -0.38 0.0  0.85  0.0 --way 0.8  1.27  -0.6 0.0  0.85  0.0 #new home
python3 arm_commander.py --arm right --pos 0.0  -0.96  1.16  0.0  -0.3  0.0 --time 4.0

