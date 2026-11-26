export INTERBOTIX_ALOHA_IS_MOBILE=true # true for Mobile, false for Stationary
source /opt/ros/humble/setup.bash # configure ROS system install environment
source ~/interbotix_ws/install/setup.bash # configure ROS workspace environment
cd ~/mauna_operation/internal/
python3 arm_commander.py --arm left --pos 0.0  -0.96  1.16  0.0  -0.3  0.0 --time 2.0
python3 arm_commander.py --arm right --pos 0.0  -0.96  1.16  0.0  -0.3  0.0 --time 4.0
python3 gripper_commander.py --arm right --pos 1.5923 --time 1.0
python3 gripper_commander.py --arm left --pos 1.5923 --time 1.0
python3 gripper_commander.py --arm right --pos 0.0185 --time 1.0
python3 gripper_commander.py --arm left --pos 0.0185 --time 1.0
python3 replay_episodes.py --dataset_dir ../mauna_data/close_lid --episode_idx 0
