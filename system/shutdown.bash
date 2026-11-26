#source ~/trossen_init.bash
source /opt/ros/humble/setup.bash
source ~/interbotix_ws/install/setup.bash
cd ~/mauna_operation/internal

python3 arm_commander.py --arm left --pos 0.0  -0.96  1.16  0.0  -0.3  0.0 --time 4.0 &
python3 arm_commander.py --arm right --pos 0.0  -0.96  1.16  0.0  -0.3  0.0 --time 4.0
python3 arm_commander.py --arm right --time 4.0 --pos 0.0 -1.8 1.55 0.0 -1.57 0.0 &
python3 arm_commander.py --arm left --time 4.0 --pos 0.0 -1.8 1.55 0.0 -1.57 0.0
