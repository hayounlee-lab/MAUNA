import argparse
from copy import deepcopy
from itertools import repeat
import os
import pickle
import time

from aloha.constants import (
    FPS,
    DT,
    FOLLOWER_GRIPPER_JOINT_CLOSE,
    FOLLOWER_GRIPPER_JOINT_OPEN,
    FOLLOWER_GRIPPER_JOINT_UNNORMALIZE_FN,
    FOLLOWER_GRIPPER_POSITION_NORMALIZE_FN,
    FOLLOWER_GRIPPER_VELOCITY_NORMALIZE_FN,
    IS_MOBILE,
    LEADER_GRIPPER_JOINT_NORMALIZE_FN,
    START_ARM_POSE,
    TASK_CONFIGS,
    DATA_DIR,
)
from einops import rearrange
import matplotlib.pyplot as plt
import numpy as np
import torch
from torchvision import transforms
from tqdm import tqdm
import wandb

from detr.models.latent_model import Latent_Model_Transformer
from policy import (
    ACTPolicy,
    CNNMLPPolicy,
    DiffusionPolicy
)
from utils import (
    load_data,
    compute_dict_mean,
    set_seed,
)

import collections

from aloha.robot_utils import (
    ImageRecorder,
    move_arms,
    move_grippers,
    Recorder,
    setup_follower_bot,
    setup_leader_bot,
)
import dm_env
from interbotix_common_modules.common_robot.robot import (
    create_interbotix_global_node,
    get_interbotix_global_node,
    InterbotixRobotNode,
    robot_startup,
)
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_xs_modules.xs_robot.slate import InterbotixSlate
from interbotix_xs_msgs.msg import JointGroupCommand, JointSingleCommand
import matplotlib.pyplot as plt


class RealEnv:
    """
    Environment for real robot bi-manual manipulation.

    Action space: [
        left_arm_qpos (6),             # absolute joint position
        left_gripper_positions (1),    # normalized gripper position (0: close, 1: open)
        right_arm_qpos (6),            # absolute joint position
        right_gripper_positions (1),   # normalized gripper position (0: close, 1: open)
    ]

    Observation space: {
        "qpos": Concat[
            left_arm_qpos (6),          # absolute joint position
            left_gripper_position (1),  # normalized gripper position (0: close, 1: open)
            right_arm_qpos (6),         # absolute joint position
            right_gripper_qpos (1)      # normalized gripper position (0: close, 1: open)
        ]
        "qvel": Concat[
            left_arm_qvel (6),          # absolute joint velocity (rad)
            left_gripper_velocity (1),  # normalized gripper velocity (pos: opening, neg: closing)
            right_arm_qvel (6),         # absolute joint velocity (rad)
            right_gripper_qvel (1)      # normalized gripper velocity (pos: opening, neg: closing)
        ]
        "images": {
            "cam_high": (480x640x3),        # h, w, c, dtype='uint8'
            "cam_low": (480x640x3),         # h, w, c, dtype='uint8'
            "cam_left_wrist": (480x640x3),  # h, w, c, dtype='uint8'
            "cam_right_wrist": (480x640x3)  # h, w, c, dtype='uint8'
        }
    """

    def __init__(
        self,
        node: InterbotixRobotNode,
        setup_robots: bool = True,
        setup_base: bool = False,
        is_mobile: bool = IS_MOBILE,
    ):
        self.follower_bot_left = InterbotixManipulatorXS(
            robot_model='vx300s',
            group_name='arm',
            gripper_name='gripper',
            robot_name='follower_left',
            node=node,
            iterative_update_fk=False,
        )
        self.follower_bot_right = InterbotixManipulatorXS(
            robot_model='vx300s',
            group_name='arm',
            gripper_name='gripper',
            robot_name='follower_right',
            node=node,
            iterative_update_fk=False,
        )

        self.recorder_left = Recorder('left', node=node)
        self.recorder_right = Recorder('right', node=node)
        self.image_recorder = ImageRecorder(node=node, is_mobile=IS_MOBILE, is_debug=True)
        self.gripper_command = JointSingleCommand(name='gripper')

        self.pub_left_arm = node.create_publisher(JointGroupCommand, '/cmd/left_arm', 10)
        self.pub_left_gripper = node.create_publisher(JointSingleCommand, '/cmd/left_gripper', 10)

        self.pub_right_arm = node.create_publisher(JointGroupCommand, '/cmd/right_arm', 10)
        self.pub_right_gripper = node.create_publisher(JointSingleCommand, '/cmd/right_gripper', 10)

        if setup_robots:
            self.setup_robots()

        if setup_base:
            if is_mobile:
                self.setup_base(node)
            else:
                raise ValueError((
                    'Requested to set up base but robot is not mobile. '
                    "Hint: check the 'IS_MOBILE' constant."
                ))

    def setup_base(self, node):
        self.base = InterbotixSlate(
            'aloha',
            node=node,
        )
        self.base.base.set_motor_torque(False)

    def setup_robots(self):
        setup_follower_bot(self.follower_bot_left)
        setup_follower_bot(self.follower_bot_right)

    def get_qpos(self):
        left_qpos_raw = self.recorder_left.qpos
        right_qpos_raw = self.recorder_right.qpos
        left_arm_qpos = left_qpos_raw[:6]
        right_arm_qpos = right_qpos_raw[:6]
        left_gripper_qpos = [FOLLOWER_GRIPPER_POSITION_NORMALIZE_FN(left_qpos_raw[7])]
        right_gripper_qpos = [FOLLOWER_GRIPPER_POSITION_NORMALIZE_FN(right_qpos_raw[7])]
        return np.concatenate(
            [left_arm_qpos, left_gripper_qpos, right_arm_qpos, right_gripper_qpos]
        )

    def get_qvel(self):
        left_qvel_raw = self.recorder_left.qvel
        right_qvel_raw = self.recorder_right.qvel
        left_arm_qvel = left_qvel_raw[:6]
        right_arm_qvel = right_qvel_raw[:6]
        left_gripper_qvel = [FOLLOWER_GRIPPER_VELOCITY_NORMALIZE_FN(left_qvel_raw[7])]
        right_gripper_qvel = [FOLLOWER_GRIPPER_VELOCITY_NORMALIZE_FN(right_qvel_raw[7])]
        return np.concatenate(
            [left_arm_qvel, left_gripper_qvel, right_arm_qvel, right_gripper_qvel]
        )

    def get_effort(self):
        left_effort_raw = self.recorder_left.effort
        right_effort_raw = self.recorder_right.effort
        left_robot_effort = left_effort_raw[:7]
        right_robot_effort = right_effort_raw[:7]
        return np.concatenate([left_robot_effort, right_robot_effort])

    def get_images(self):
        return self.image_recorder.get_images()

    def get_base_vel(self):
        linear_vel = self.base.base.get_linear_velocity().x
        angular_vel = self.base.base.get_angular_velocity().z
        return np.array([linear_vel, angular_vel])

    def set_gripper_pose(
        self,
        left_gripper_desired_pos_normalized,
        right_gripper_desired_pos_normalized
    ):
        left_gripper_desired_joint = FOLLOWER_GRIPPER_JOINT_UNNORMALIZE_FN(
            left_gripper_desired_pos_normalized
        )
        self.gripper_command.cmd = left_gripper_desired_joint
        self.follower_bot_left.gripper.core.pub_single.publish(self.gripper_command)

        right_gripper_desired_joint = FOLLOWER_GRIPPER_JOINT_UNNORMALIZE_FN(
            right_gripper_desired_pos_normalized
        )
        self.gripper_command.cmd = right_gripper_desired_joint
        self.follower_bot_right.gripper.core.pub_single.publish(self.gripper_command)

    def _reset_joints(self):
        reset_position = START_ARM_POSE[:6]
        move_arms(
            [self.follower_bot_left, self.follower_bot_right],
            [reset_position, reset_position],
            moving_time=1.0,
        )

    def _reset_gripper(self):
        """
        Set to position mode and do position resets.

        First open then close, then change back to PWM mode
        """
        move_grippers(
            [self.follower_bot_left, self.follower_bot_right],
            [FOLLOWER_GRIPPER_JOINT_OPEN] * 2,
            moving_time=0.5,
        )
        move_grippers(
            [self.follower_bot_left, self.follower_bot_right],
            [FOLLOWER_GRIPPER_JOINT_CLOSE] * 2,
            moving_time=1.0,
        )

    def get_observation(self, get_base_vel=False):
        obs = collections.OrderedDict()
        obs['qpos'] = self.get_qpos()
        obs['qvel'] = self.get_qvel()
        obs['effort'] = self.get_effort()
        obs['images'] = self.get_images()
        #obs['base_vel'] = self.get_base_vel()
        if get_base_vel:
            obs['base_vel'] = self.get_base_vel()
        return obs

    def get_reward(self):
        return 0

    def reset(self, fake=False, get_base_vel=False):
        if not fake:
            # Reboot follower robot gripper motors
            self.follower_bot_left.core.robot_reboot_motors('single', 'gripper', True)
            self.follower_bot_right.core.robot_reboot_motors('single', 'gripper', True)
            self._reset_joints()
            self._reset_gripper()
        return dm_env.TimeStep(
            step_type=dm_env.StepType.FIRST,
            reward=self.get_reward(),
            discount=None,
            observation=self.get_observation(get_base_vel),
        )

    def step(self, action, base_action=None, get_base_vel=False, get_obs=True):
        state_len = int(len(action) / 2)
        left_action = action[:state_len]
        right_action = action[state_len:]
        
        # Left Arm & Gripper
        msg_left_arm = JointGroupCommand()
        msg_left_arm.cmd = left_action[:6].tolist() if hasattr(left_action, 'tolist') else left_action[:6]
        
        msg_left_gripper = JointSingleCommand()
        msg_left_gripper.cmd = float(left_action[-1])

        # Right Arm & Gripper
        msg_right_arm = JointGroupCommand()
        msg_right_arm.cmd = right_action[:6].tolist() if hasattr(right_action, 'tolist') else right_action[:6]
        
        msg_right_gripper = JointSingleCommand()
        msg_right_gripper.cmd = float(right_action[-1])

        self.pub_left_arm.publish(msg_left_arm)
        self.pub_left_gripper.publish(msg_left_gripper)
        self.pub_right_arm.publish(msg_right_arm)
        self.pub_right_gripper.publish(msg_right_gripper)

        if base_action is not None:
            base_action_linear, base_action_angular = base_action
            self.base.base.command_velocity_xyaw(x=base_action_linear, yaw=base_action_angular)
        if get_obs:
            obs = self.get_observation(get_base_vel)
        else:
            obs = None
        return dm_env.TimeStep(
            step_type=dm_env.StepType.MID,
            reward=self.get_reward(),
            discount=None,
            observation=obs)


def make_real_env(
    node: InterbotixRobotNode = None,
    setup_robots: bool = True,
    setup_base: bool = False
):
    if node is None:
        node = get_interbotix_global_node()
        if node is None:
            node = create_interbotix_global_node('aloha')
    env = RealEnv(node, setup_robots, setup_base)
    return env

def get_auto_index(dataset_dir):
    max_idx = 1000
    for i in range(max_idx+1):
        if not os.path.isfile(os.path.join(dataset_dir, f'qpos_{i}.npy')):
            return i
    raise Exception(f"Error getting auto index, or more than {max_idx} episodes")


def main(args):
    set_seed(1)
    # command line parameters
    is_eval = args['eval']
    ckpt_dir = args['ckpt_dir']
    policy_class = args['policy_class']
    onscreen_render = args['onscreen_render']
    task_name = args['task_name']
    batch_size_train = args['batch_size']
    batch_size_val = args['batch_size']
    num_steps = args['num_steps']
    validate_every = args['validate_every']
    save_every = args['save_every']
    resume_ckpt_path = args['resume_ckpt_path']

    # get task parameters
    task_configs = TASK_CONFIGS
    task_configs['check_plate']={
        'dataset_dir': DATA_DIR + '/check_plate',
        'episode_len': 400,
        'camera_names': ['cam_high', 'cam_left_wrist', 'cam_right_wrist']
    }
    task_configs['check_sign']={
        'dataset_dir': DATA_DIR + '/check_sign',
        'episode_len': 400,
        'camera_names': ['cam_high', 'cam_left_wrist', 'cam_right_wrist']
    }
    task_configs['press_button']={
        'dataset_dir': DATA_DIR + '/press_button',
        'episode_len': 1200,
        'camera_names': ['cam_high', 'cam_left_wrist', 'cam_right_wrist']
    }
    task_configs['inspect_tube']={
        'dataset_dir': DATA_DIR + '/inspect_tube',
        'episode_len': 400,
        'camera_names': ['cam_high', 'cam_left_wrist', 'cam_right_wrist']
    }
    task_config = task_configs[args['task_name']]
    dataset_dir = task_config['dataset_dir']
    episode_len = task_config['episode_len']
    camera_names = task_config['camera_names']
    stats_dir = task_config.get('stats_dir', None)
    sample_weights = task_config.get('sample_weights', None)
    train_ratio = task_config.get('train_ratio', 0.99)
    name_filter = task_config.get('name_filter', lambda n: True)

    # fixed parameters
    state_dim = 14
    lr_backbone = 1e-5
    backbone = 'resnet18'
    if policy_class == 'ACT':
        enc_layers = 4
        dec_layers = 7
        nheads = 8
        policy_config = {
            'action_dim': 16,
            'backbone': backbone,
            'camera_names': camera_names,
            'dec_layers': dec_layers,
            'dim_feedforward': args['dim_feedforward'],
            'enc_layers': enc_layers,
            'hidden_dim': args['hidden_dim'],
            'kl_weight': args['kl_weight'],
            'lr_backbone': lr_backbone,
            'lr': args['lr'],
            'nheads': nheads,
            'no_encoder': args['no_encoder'],
            'num_queries': args['chunk_size'],
            'vq_class': args['vq_class'],
            'vq_dim': args['vq_dim'],
            'vq': args['use_vq'],
            'adapter_dim': args['adapter_dim'],
            'use_base': args['use_base'],
        }
    elif policy_class == 'Diffusion':
        policy_config = {
            'action_dim': 16,
            'action_horizon': 8,
            'camera_names': camera_names,
            'ema_power': 0.75,
            'lr': args['lr'],
            'num_inference_timesteps': 10,
            'num_queries': args['chunk_size'],
            'observation_horizon': 1,
            'prediction_horizon': args['chunk_size'],
            'vq': False,
        }
    elif policy_class == 'CNNMLP':
        policy_config = {
            'backbone' : backbone,
            'camera_names': camera_names,
            'lr_backbone': lr_backbone,
            'lr': args['lr'],
            'num_queries': 1,
        }
    else:
        raise NotImplementedError

    actuator_config = {
        'actuator_network_dir': args['actuator_network_dir'],
        'history_len': args['history_len'],
        'future_len': args['future_len'],
        'prediction_len': args['prediction_len'],
    }

    config = {
        'actuator_config': actuator_config,
        'camera_names': camera_names,
        'ckpt_dir': ckpt_dir,
        'episode_len': episode_len,
        'lr': args['lr'],
        'num_steps': num_steps,
        'onscreen_render': onscreen_render,
        'policy_class': policy_class,
        'policy_config': policy_config,
        'real_robot': True,
        'resume_ckpt_path': resume_ckpt_path,
        'save_every': save_every,
        'seed': args['seed'],
        'state_dim': state_dim,
        'task_name': task_name,
        'temporal_agg': args['temporal_agg'],
        'validate_every': validate_every,
        'use_base': args['use_base'],
    }

    if not os.path.isdir(ckpt_dir):
        os.makedirs(ckpt_dir)
    config_path = os.path.join(ckpt_dir, 'config.pkl')
    expr_name = ckpt_dir.split('/')[-1]
    if not is_eval:
        wandb.init(
            project="ExampleProject",
            reinit=True,
            entity="ExampleEntity",
            name=expr_name,
        )
        wandb.config.update(config)
    with open(config_path, 'wb') as f:
        pickle.dump(config, f)
    if is_eval:
        ckpt_names = ['policy_best.ckpt']
        results = []
        for ckpt_name in ckpt_names:
            success_rate, avg_return = eval_bc(
                config,
                ckpt_name,
                save_episode=True,
                num_rollouts=1,
            )
            results.append([ckpt_name, success_rate, avg_return])

        for ckpt_name, success_rate, avg_return in results:
            print(f'{ckpt_name}: {success_rate=} {avg_return=}')
        print()
        os._exit(0)

    train_dataloader, val_dataloader, stats, _ = load_data(
        dataset_dir,
        name_filter,
        camera_names,
        batch_size_train,
        batch_size_val,
        args['chunk_size'],
        args['skip_mirrored_data'],
        policy_class,
        stats_dir_l=stats_dir,
        sample_weights=sample_weights,
        train_ratio=train_ratio,
    )

    # save dataset stats
    stats_path = os.path.join(ckpt_dir, 'dataset_stats.pkl')
    with open(stats_path, 'wb') as f:
        pickle.dump(stats, f)

    best_ckpt_info = train_bc(train_dataloader, val_dataloader, config)
    best_step, min_val_loss, best_state_dict = best_ckpt_info

    # save best checkpoint
    ckpt_path = os.path.join(ckpt_dir, 'policy_best.ckpt')
    torch.save(best_state_dict, ckpt_path)
    print(f'Best ckpt, val loss {min_val_loss:.6f} @ step{best_step}')
    wandb.finish()


def make_policy(policy_class, policy_config):
    if policy_class == 'ACT':
        policy = ACTPolicy(policy_config)
    elif policy_class == 'CNNMLP':
        policy = CNNMLPPolicy(policy_config)
    elif policy_class == 'Diffusion':
        policy = DiffusionPolicy(policy_config)
    else:
        raise NotImplementedError
    return policy


def make_optimizer(policy_class, policy):
    if policy_class == 'ACT':
        optimizer = policy.configure_optimizers()
    elif policy_class == 'CNNMLP':
        optimizer = policy.configure_optimizers()
    elif policy_class == 'Diffusion':
        optimizer = policy.configure_optimizers()
    else:
        raise NotImplementedError
    return optimizer

def get_image(ts, camera_names, rand_crop_resize=False, custom_crop_boxes=None):
    """
    - rand_crop_resize: Original random crop/resize option (True/False)
    - custom_crop_boxes: Dictionary specifying custom crop boxes per camera
                         e.g., {'cam_high': (0.25, 0.25, 0.75, 0.75), 'cam_left_wrist': None}
    """
    curr_images = []
    for cam_name in camera_names:
        img_np = rearrange(ts.observation['images'][cam_name], 'h w c -> c h w')
        img_tensor = torch.from_numpy(img_np / 255.0).float().cuda()
        
        if custom_crop_boxes is not None and cam_name in custom_crop_boxes and custom_crop_boxes[cam_name] is not None:
            box = custom_crop_boxes[cam_name]
            h, w = img_tensor.shape[-2:]  # Original resolution (H, W)
            print(h,w)
            y1, y2 = int(h * box[0]), int(h * box[2])
            x1, x2 = int(w * box[1]), int(w * box[3])
            
            # Crop specific region and resize back to original resolution
            img_tensor = img_tensor[..., y1:y2, x1:x2]
            resize_transform = transforms.Resize((h, w), antialias=True)
            img_tensor = resize_transform(img_tensor)
            h, w = img_tensor.shape[-2:]  # Original resolution (H, W)
            
        curr_images.append(img_tensor)
        
    curr_image = torch.stack(curr_images, dim=0).unsqueeze(0)

    if rand_crop_resize:
        print('rand crop resize is used!')
        original_size = curr_image.shape[-2:]
        ratio = 0.95
        curr_image = curr_image[..., int(original_size[0] * (1 - ratio) / 2): int(original_size[0] * (1 + ratio) / 2),
                                    int(original_size[1] * (1 - ratio) / 2): int(original_size[1] * (1 + ratio) / 2)]
        curr_image = curr_image.squeeze(0)
        resize_transform = transforms.Resize(original_size, antialias=True)
        curr_image = resize_transform(curr_image)
        curr_image = curr_image.unsqueeze(0)

    return curr_image



def eval_bc(config, ckpt_name, save_episode=True, num_rollouts=50):
    set_seed(1000)
    ckpt_dir = config['ckpt_dir']
    state_dim = config['state_dim']
    real_robot = config['real_robot']
    policy_class = config['policy_class']
    onscreen_render = config['onscreen_render']
    policy_config = config['policy_config']
    camera_names = config['camera_names']
    max_timesteps = config['episode_len']
    task_name = config['task_name']
    temporal_agg = config['temporal_agg']
    onscreen_cam = 'angle'
    vq = config['policy_config']['vq']
    actuator_config = config['actuator_config']
    use_base = config['use_base']

    # load policy and stats
    ckpt_path = os.path.join(ckpt_dir, ckpt_name)
    policy = make_policy(policy_class, policy_config)
    loading_status = policy.deserialize(torch.load(ckpt_path))
    print(loading_status)
    policy.cuda()
    policy.eval()
    if vq:
        vq_dim = config['policy_config']['vq_dim']
        vq_class = config['policy_config']['vq_class']
        latent_model = Latent_Model_Transformer(vq_dim, vq_dim, vq_class)
        latent_model_ckpt_path = os.path.join(ckpt_dir, 'latent_model_last.ckpt')
        latent_model.deserialize(torch.load(latent_model_ckpt_path))
        latent_model.eval()
        latent_model.cuda()
        print(f'Loaded policy from: {ckpt_path}, latent model from: {latent_model_ckpt_path}')
    else:
        print(f'Loaded: {ckpt_path}')
    stats_path = os.path.join(ckpt_dir, f'dataset_stats.pkl')
    with open(stats_path, 'rb') as f:
        stats = pickle.load(f)

    pre_process = lambda s_qpos: (s_qpos - stats['qpos_mean']) / stats['qpos_std']
    if policy_class == 'Diffusion':
        post_process = lambda a: ((a + 1) / 2) * (stats['action_max'] - stats['action_min']) + stats['action_min']
    else:
        post_process = lambda a: a * stats['action_std'] + stats['action_mean']

    from interbotix_common_modules.common_robot.exceptions import InterbotixException
    try:
        node = get_interbotix_global_node()
    except:
        node = create_interbotix_global_node('aloha')
    env = make_real_env(node=node, setup_robots=False, setup_base=use_base)
    if use_base:
       env.base.base.set_motor_torque(True)
    try:
        robot_startup(node)
    except InterbotixException:
        pass
    time.sleep(2.)
    env_max_reward = 0

    query_frequency = policy_config['num_queries']
    if temporal_agg:
        query_frequency = 1
        num_queries = policy_config['num_queries']
    if real_robot:
        BASE_DELAY = 13
        query_frequency -= BASE_DELAY

    max_timesteps = int(max_timesteps * 1) # may increase for real-world tasks

    episode_returns = []
    highest_rewards = []
    for rollout_id in range(num_rollouts):
        rollout_id += 0

        reset_not_done=True
        while reset_not_done:
           try:
              ts = env.reset(fake=True)
              reset_not_done=False
           except:
              continue
            

        ### onscreen render
        if onscreen_render:
            ax = plt.subplot()
            plt_img = ax.imshow(env._physics.render(height=480, width=640, camera_id=onscreen_cam))
            plt.ion()

        # evaluation loop
        if temporal_agg:
            all_time_actions = torch.zeros([max_timesteps, max_timesteps+num_queries, 16]).cuda()

        qpos_history_raw = np.zeros((max_timesteps, state_dim))
        image_list = [] # for visualization
        qpos_list = []
        target_qpos_list = []
        rewards = []
        with torch.inference_mode():
            time0 = time.time()
            DT = 1 / FPS
            culmulated_delay = 0
            for t in range(max_timesteps):
                time1 = time.time()
                ### update onscreen render and wait for DT
                if onscreen_render:
                    image = env._physics.render(height=480, width=640, camera_id=onscreen_cam)
                    plt_img.set_data(image)
                    plt.pause(DT)

                ### process previous timestep to get qpos and image_list
                time2 = time.time()
                obs = ts.observation
                if 'images' in obs:
                    image_list.append(obs['images'])
                else:
                    image_list.append({'main': obs['image']})
                qpos_numpy = np.array(obs['qpos'])
                qpos_history_raw[t] = qpos_numpy
                qpos = pre_process(qpos_numpy)
                qpos = torch.from_numpy(qpos).float().cuda().unsqueeze(0)
                if t % query_frequency == 0:
                    my_crop_boxes = {
                       'cam_high': (0.25, 0.5, 0.75, 1.0),   
                       'cam_left_wrist': (0.5, 0.25, 1.0, 0.75),
                       'cam_right_wrist': (0.1, 0.37, 0.45, 0.74)
                    }
                    curr_image = get_image(
                        ts,
                        camera_names,
                        rand_crop_resize=(config['policy_class'] == 'Diffusion'),
                        custom_crop_boxes=my_crop_boxes
                    )

                if t == 0:
                    # warm up
                    for _ in range(10):
                        policy(qpos, curr_image)
                    print('network warm up done')
                    time1 = time.time()

                ### query policy
                time3 = time.time()
                if config['policy_class'] == "ACT":
                    if t % query_frequency == 0:
                        if vq:
                            if rollout_id == 0:
                                for _ in range(10):
                                    vq_sample = latent_model.generate(1, temperature=1, x=None)
                                    print(torch.nonzero(vq_sample[0])[:, 1].cpu().numpy())
                            vq_sample = latent_model.generate(1, temperature=1, x=None)
                            all_actions = policy(qpos, curr_image, vq_sample=vq_sample)
                        else:
                            all_actions = policy(qpos, curr_image)
                        if real_robot:
                            all_actions = torch.cat(
                                [
                                    all_actions[:, :-BASE_DELAY, :-2],
                                    all_actions[:, BASE_DELAY:, -2:]
                                ],
                                dim=2
                            )
                    if temporal_agg:
                        all_time_actions[[t], t:t+num_queries-BASE_DELAY] = all_actions
                        actions_for_curr_step = all_time_actions[:, t]
                        actions_populated = torch.all(actions_for_curr_step != 0, axis=1)
                        actions_for_curr_step = actions_for_curr_step[actions_populated]
                        k = 0.01
                        exp_weights = np.exp(-k * np.arange(len(actions_for_curr_step)))
                        exp_weights = exp_weights / exp_weights.sum()
                        exp_weights = torch.from_numpy(exp_weights).cuda().unsqueeze(dim=1)
                        raw_action = (actions_for_curr_step * exp_weights).sum(dim=0, keepdim=True)
                    else:
                        raw_action = all_actions[:, t % query_frequency]
                elif config['policy_class'] == "Diffusion":
                    if t % query_frequency == 0:
                        all_actions = policy(qpos, curr_image)
                        if real_robot:
                            all_actions = torch.cat([all_actions[:, :-BASE_DELAY, :-2], all_actions[:, BASE_DELAY:, -2:]], dim=2)
                    raw_action = all_actions[:, t % query_frequency]
                elif config['policy_class'] == "CNNMLP":
                    raw_action = policy(qpos, curr_image)
                    all_actions = raw_action.unsqueeze(0)
                else:
                    raise NotImplementedError

                ### post-process actions
                raw_action = raw_action.squeeze(0).cpu().numpy()
                action = post_process(raw_action)
                target_qpos = action[:-2]

                base_action = action[-2:]

                # step the environment
                if real_robot and use_base:
                    ts = env.step(target_qpos, base_action)
                else:
                    ts = env.step(target_qpos)

                ### for visualization
                qpos_list.append(qpos_numpy)
                target_qpos_list.append(target_qpos)
                rewards.append(ts.reward)
                duration = time.time() - time1
                sleep_time = max(0, DT - duration)
                time.sleep(sleep_time)
                if duration >= DT:
                    culmulated_delay += (duration - DT)
                    print((
                        f'Warning: step duration: {duration:.3f} s at step {t} longer than DT: '
                        f'{DT} s, culmulated delay: {culmulated_delay:.3f} s'
                    ))

            print(f'Avg fps {max_timesteps / (time.time() - time0)}')
            plt.close()
        if real_robot:
            #move_grippers(
            #    [env.follower_bot_left, env.follower_bot_right],
            #    [FOLLOWER_GRIPPER_JOINT_OPEN] * 2,
            #    moving_time=0.5,
            #)  # open
            # save qpos_history_raw
            log_id = get_auto_index(ckpt_dir)
            np.save(os.path.join(ckpt_dir, f'qpos_{log_id}.npy'), qpos_history_raw)
            plt.figure(figsize=(10, 20))
            # plot qpos_history_raw for each qpos dim using subplots
            for i in range(state_dim):
                plt.subplot(state_dim, 1, i+1)
                plt.plot(qpos_history_raw[:, i])
                # remove x axis
                if i != state_dim - 1:
                    plt.xticks([])
            plt.tight_layout()
            plt.savefig(os.path.join(ckpt_dir, f'qpos_{log_id}.png'))
            plt.close()


        rewards = np.array(rewards)
        episode_return = np.sum(rewards[rewards!=None])
        episode_returns.append(episode_return)
        episode_highest_reward = np.max(rewards)
        highest_rewards.append(episode_highest_reward)
        print((
            f'Rollout {rollout_id}\n{episode_return=}, {episode_highest_reward=}, '
            f'{env_max_reward=}, Success: {episode_highest_reward==env_max_reward}'
        ))

    success_rate = np.mean(np.array(highest_rewards) == env_max_reward)
    avg_return = np.mean(episode_returns)
    summary_str = f'\nSuccess rate: {success_rate}\nAverage return: {avg_return}\n\n'
    for r in range(env_max_reward+1):
        more_or_equal_r = (np.array(highest_rewards) >= r).sum()
        more_or_equal_r_rate = more_or_equal_r / num_rollouts
        summary_str += f'Reward >= {r}: {more_or_equal_r}/{num_rollouts} = {more_or_equal_r_rate*100}%\n'

    print(summary_str)

    # save success rate to txt
    result_file_name = 'result_' + ckpt_name.split('.')[0] + '.txt'
    with open(os.path.join(ckpt_dir, result_file_name), 'w') as f:
        f.write(summary_str)
        f.write(repr(episode_returns))
        f.write('\n\n')
        f.write(repr(highest_rewards))

    return success_rate, avg_return


def forward_pass(data, policy):
    image_data, qpos_data, action_data, is_pad = data

    image_data = image_data.cuda()
    qpos_data = qpos_data.cuda()
    action_data = action_data.cuda()
    is_pad = is_pad.cuda()

    return policy(qpos_data, image_data, action_data, is_pad)


def train_bc(train_dataloader, val_dataloader, config):
    num_steps = config['num_steps']
    ckpt_dir = config['ckpt_dir']
    seed = config['seed']
    policy_class = config['policy_class']
    policy_config = config['policy_config']
    validate_every = config['validate_every']
    save_every = config['save_every']

    set_seed(seed)

    policy = make_policy(policy_class, policy_config)
    if config['resume_ckpt_path'] is not None:
        if policy_class == 'ACT' and policy_config.get('adapter_dim',None) is not None:
           loading_status = policy.deserialize(torch.load(config['resume_ckpt_path']),allow_adapter_mismatch=True, verbose=True)
        else:
           loading_status = policy.deserialize(torch.load(config['resume_ckpt_path']))
        print(f'Resume policy from: {config["resume_ckpt_path"]}, Status: {loading_status}')
    policy.cuda()
    optimizer = make_optimizer(policy_class, policy)

    min_val_loss = np.inf
    best_ckpt_info = None

    train_dataloader = repeater(train_dataloader)
    for step in tqdm(range(num_steps+1)):
        # validation
        if step % validate_every == 0:
            print('validating')

            with torch.inference_mode():
                policy.eval()
                validation_dicts = []
                for batch_idx, data in enumerate(val_dataloader):
                    forward_dict = forward_pass(data, policy)
                    validation_dicts.append(forward_dict)
                    if batch_idx > 50:
                        break

                validation_summary = compute_dict_mean(validation_dicts)

                epoch_val_loss = validation_summary['loss']
                if epoch_val_loss < min_val_loss:
                    min_val_loss = epoch_val_loss
                    best_ckpt_info = (step, min_val_loss, deepcopy(policy.serialize()))
            for k in list(validation_summary.keys()):
                validation_summary[f'val_{k}'] = validation_summary.pop(k)
            wandb.log(validation_summary, step=step)
            print(f'Val loss:   {epoch_val_loss:.5f}')
            summary_string = ''
            for k, v in validation_summary.items():
                summary_string += f'{k}: {v.item():.3f} '
            print(summary_string)

        # training
        policy.train()
        optimizer.zero_grad()
        data = next(train_dataloader)
        forward_dict = forward_pass(data, policy)
        # backward
        loss = forward_dict['loss']
        loss.backward()
        optimizer.step()
        wandb.log(forward_dict, step=step) # not great, make training 1-2% slower

        if step % save_every == 0:
            ckpt_path = os.path.join(ckpt_dir, f'policy_step_{step}_seed_{seed}.ckpt')
            torch.save(policy.serialize(), ckpt_path)

    ckpt_path = os.path.join(ckpt_dir, 'policy_best.ckpt')
    torch.save(policy.serialize(), ckpt_path)

    best_step, min_val_loss, best_state_dict = best_ckpt_info
    ckpt_path = os.path.join(ckpt_dir, f'policy_step_{best_step}_seed_{seed}.ckpt')
    torch.save(best_state_dict, ckpt_path)
    print(f'Training finished:\nSeed {seed}, val loss {min_val_loss:.6f} at step {best_step}')

    return best_ckpt_info


def repeater(data_loader):
    epoch = 0
    for loader in repeat(data_loader):
        for data in loader:
            yield data
        print(f'Epoch {epoch} done')
        epoch += 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--eval',
        action='store_true',
        help='Evaluate the selected model checkpoint',
    )
    parser.add_argument(
        '--onscreen_render',
        action='store_true',
        help='Render training onscreen',
    )
    parser.add_argument(
        '--ckpt_dir',
        action='store',
        type=str,
        help='Checkpoint directory',
        required=True,
    )
    parser.add_argument(
        '--policy_class',
        action='store',
        type=str,
        default='ACT',
        help='The desired policy class',
        choices=['ACT', 'Diffusion', 'CNNMLP'],
    )
    parser.add_argument(
        '--task_name',
        action='store',
        type=str,
        help='Name of the task. Must be in task configurations',
        required=True
    )
    parser.add_argument(
        '--batch_size',
        action='store',
        type=int,
        help='Training batch size',
        required=True
    )
    parser.add_argument(
        '--seed',
        action='store',
        type=int,
        help='Training seed',
        required=True
    )
    parser.add_argument(
        '--num_steps',
        action='store',
        type=int,
        help='Number of training steps',
        required=True
    )
    parser.add_argument(
        '--lr',
        action='store',
        type=float,
        help='Training learning rate',
        required=True
    )
    parser.add_argument(
        '--validate_every',
        action='store',
        type=int,
        default=500,
        help='Number of steps between validations during training',
        required=False,
    )
    parser.add_argument(
        '--save_every',
        action='store',
        type=int,
        default=500,
        help='Number of steps between checkpoints during training',
        required=False,
    )
    parser.add_argument(
        '--resume_ckpt_path',
        action='store',
        type=str,
        help='Path to checkpoint to resume training from',
        required=False,
    )
    parser.add_argument(
        '--skip_mirrored_data',
        action='store_true',
        help='Skip mirrored data during training',
        required=False,
    )
    parser.add_argument(
        '--actuator_network_dir',
        action='store',
        type=str,
        help='actuator_network_dir',
        required=False,
    )
    parser.add_argument(
        '--history_len',
        action='store',
        type=int,
    )
    parser.add_argument(
        '--future_len',
        action='store',
        type=int,
    )
    parser.add_argument(
        '--prediction_len',
        action='store',
        type=int,
    )

    # for ACT
    parser.add_argument(
        '--kl_weight',
        action='store',
        type=int,
        help='KL Weight',
        required=False,
    )
    parser.add_argument(
        '--chunk_size',
        action='store',
        type=int,
        help='chunk_size',
        required=False,
    )
    parser.add_argument(
        '--hidden_dim',
        action='store',
        type=int,
        help='hidden_dim',
        required=False,
    )
    parser.add_argument(
        '--dim_feedforward',
        action='store',
        type=int,
        help='dim_feedforward',
        required=False,
    )
    parser.add_argument(
        '--temporal_agg',
        action='store_true',
    )
    parser.add_argument(
        '--use_vq',
        action='store_true',
    )
    parser.add_argument(
        '--vq_class',
        action='store',
        type=int,
        help='vq_class',
    )
    parser.add_argument(
        '--vq_dim',
        action='store',
        type=int,
        help='vq_dim',
    )
    parser.add_argument(
        '--no_encoder',
        action='store_true',
    )
    parser.add_argument(
        '--use_base',
        action='store_true',
        default=False,
    )
    parser.add_argument(
        '--adapter_dim',
        action='store',
        type=int,
        default=None,
        help='adapter_dim',
    )

    main(vars(parser.parse_args()))
