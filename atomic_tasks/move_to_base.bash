source /opt/ros/humble/setup.bash
source ~/interbotix_ws/install/setup.bash

if ! bash ~/mauna_operation/internal/slate_on.bash; then
   exit 1
fi
source ~/conda_init.bash
conda activate aloha

cd ~/act_training_evaluation/act_plus_plus/
python3 imitate_episodes_mobile.py \
  --task_name move_center \
  --ckpt_dir results/training_move_center \
  --policy_class ACT \
  --kl_weight 10 \
  --chunk_size 100 \
  --hidden_dim 512 \
  --batch_size 4 \
  --dim_feedforward 3200 \
  --num_steps 4000 \
  --lr 1e-5 \
  --seed 1234 \
  --eval \
  --temporal_agg 

cd ~/mauna_operation/internal/
python3 gripper_commander.py --arm right --pos 1.5923 --time 1.0
python3 gripper_commander.py --arm left --pos 1.5923 --time 1.0
python3 gripper_commander.py --arm right --pos 0.0185 --time 1.0
python3 gripper_commander.py --arm left --pos 0.0185 --time 1.0
bash ~/mauna_operation/internal/slate_off.bash
