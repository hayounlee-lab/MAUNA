export INTERBOTIX_ALOHA_IS_MOBILE=true # true for Mobile, false for Stationary
source /opt/ros/humble/setup.bash # configure ROS system install environment
source ~/interbotix_ws/install/setup.bash # configure ROS workspace environment
cd ~/mauna_operation/internal
python3 arm_commander.py --arm right --time 6.0 --pos -0.1994 0.2608 0.4249 -1.8853 -1.5677 0.9189

python3 gripper_commander.py --arm right --pos 1.5923 --time 1.0
python3 gripper_commander.py --arm right --pos 0.0185 --time 1.0

source ../system/conda_init.bash
conda activate aloha
cd ~/act_training_evaluation/act_plus_plus/
python3 imitate_episodes.py \
  --task_name check_plate \
  --ckpt_dir results/training_check_plate \
  --policy_class ACT \
  --kl_weight 10 \
  --chunk_size 100 \
  --hidden_dim 512 \
  --batch_size 4 \
  --dim_feedforward 3200 \
  --num_steps 4000 \
  --lr 1e-5 \
  --seed 1234 \
  --eval
#  --eval \
#  --temporal_agg 

sleep 0.5
cd ~/mauna_operation/internal
python3 gripper_commander.py --arm right --pos 1.5923 --time 1.0
python3 gripper_commander.py --arm right --pos 0.0185 --time 1.0
