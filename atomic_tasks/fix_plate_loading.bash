export INTERBOTIX_ALOHA_IS_MOBILE=true # true for Mobile, false for Stationary
source /opt/ros/humble/setup.bash # configure ROS system install environment
source ~/interbotix_ws/install/setup.bash # configure ROS workspace environment
cd ~/mauna_operation/internal
python3 arm_commander.py --arm left --time 4.0 --start 0.0  0.0  0.0 0.0  1.45  1.5707 --target -0.2485 0.0629 0.0215 0.0 1.4051 1.3322 
python3 arm_commander.py --arm left --time 4.0 --start -0.2485 0.0629 0.0215 0.0 1.4051 1.3322 --way -0.6888 0.4694 -0.4556 0.0 1.4235 0.8819 --target -0.73388621  0.57016256 -0.41743555 0.0  1.318073    0.8819
python3 gripper_commander.py --arm left --pos 1.5923 --time 1.0
python3 arm_commander.py --arm left --time 4.0 --target -0.73395806  0.58726504 -0.38041202 0.0  1.26394697  0.8819 --start -0.73388621  0.57016256 -0.41743555 0.0  1.318073    0.8819
python3 gripper_commander.py --arm left --pos 1.3131 --time 1.0
python3 arm_commander.py --arm left --time 4.0 --start -0.73395806  0.58726504 -0.38041202 0.0  1.26394697  0.8819 --target -0.7340  0.4713 -0.3701  0.0000  1.2792  0.8819
python3 arm_commander.py --arm left --time 4.0 --start -0.7340  0.4713 -0.3701  0.0000  1.2792  0.8819 --target -0.73388621  0.57016256 -0.41743555 0.0  1.318073    0.8819
python3 gripper_commander.py --arm left --pos 1.5923 --time 1.0
python3 arm_commander.py --arm left --time 4.0 --target -0.73395806  0.58726504 -0.38041202 0.0  1.26394697  0.8819 --start -0.73388621  0.57016256 -0.41743555 0.0  1.318073    0.8819
python3 gripper_commander.py --arm left --pos 1.3131 --time 1.0
python3 gripper_commander.py --arm left --pos 1.5923 --time 1.0
python3 arm_commander.py --arm left --time 4.0 --start -0.73395806  0.58726504 -0.38041202 0.0  1.26394697  0.8819 --target -0.7340  0.4713 -0.3701  0.0000  1.2792  0.8819
python3 arm_commander.py --arm left --time 4.0 --target -0.2485 0.0629 0.0215 0.0 1.4051 1.3322 --way -0.6888 0.4694 -0.4556 0.0 1.4235 0.8819 --start -0.7340  0.4713 -0.3701  0.0000  1.2792  0.8819
python3 arm_commander.py --arm left --time 4.0 --target 0.0  0.0  0.0 0.0  1.45  1.5707 --start -0.2485 0.0629 0.0215 0.0 1.4051 1.3322 
