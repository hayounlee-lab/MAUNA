export INTERBOTIX_ALOHA_IS_MOBILE=true # true for Mobile, false for Stationary
source /opt/ros/humble/setup.bash # configure ROS system install environment
source ~/interbotix_ws/install/setup.bash # configure ROS workspace environment
cd ~/mauna_operation/system
bash init_robot.bash
cd ~/mauna_operation/internal
python3 arm_commander.py --arm right --time 4.0 --pos -0.1  0.33  -0.18 0.0  0.85  0.0 
python3 arm_commander.py --arm left --time 4.0 --pos -0.5 0.1608 0.2249 1.2853 -0.6677 -0.9189
python3 gripper_commander.py --arm right --pos 1.5923 --time 1.0
python3 gripper_commander.py --arm left --pos 1.5923 --time 1.0
python3 gripper_commander.py --arm right --pos 0.0185 --time 1.0
python3 gripper_commander.py --arm left --pos 0.0185 --time 1.0


source ../system/conda_init.bash
conda activate aloha
cd ~/act_training_evaluation/act_plus_plus/
python3 imitate_episodes_zoom.py \
  --task_name inspect_tube \
  --ckpt_dir results/training_inspect_tube_2 \
  --policy_class ACT \
  --kl_weight 10 \
  --chunk_size 100 \
  --hidden_dim 512 \
  --batch_size 4 \
  --dim_feedforward 3200 \
  --num_steps 20000 \
  --lr 1e-5 \
  --seed 1234 \
  --eval \
  --temporal_agg

cd ~/mauna_operation/internal/
python3 arm_commander.py --arm right --pos 0.0 -0.96 1.16 0.0 -0.3 0.0 --time 4.0
python3 gripper_commander.py --arm right --pos 1.5923 --time 1.0
python3 gripper_commander.py --arm left --pos 1.5923 --time 1.0
python3 gripper_commander.py --arm right --pos 0.0185 --time 1.0
python3 gripper_commander.py --arm left --pos 0.0185 --time 1.0
