# Python standard library imports
import argparse
import os
import select
import sys
import threading
import time
from datetime import datetime
import yaml

# Third-party library imports
import cv2
import h5py
import hydra
import matplotlib.pyplot as plt
import numpy as np
import torch
from PIL import Image
import torch.nn.functional as F
from omegaconf import OmegaConf
from torchvision.transforms.functional import to_tensor

# Computer vision model imports
from cutie.inference.inference_core import InferenceCore
from cutie.utils.get_default_model import get_default_model
from segment_anything import sam_model_registry, SamPredictor

from DexGraspVLA.planner.qwen2_5_vl_7b_robot_planner import DexGraspVLAPlanner as DexGraspVLAPlannerVL
# from DexGraspVLA.planner.dexgraspvla_planner_v2 import DexGraspVLAPlanner as DexGraspVLAPlannerOmni

from DexGraspVLA.controller.common.pytorch_util import dict_apply
from DexGraspVLA.inference_utils.utils import (show_mask,get_start_command, encode_image_to_base64, timer, log)

from Galaxea_R1_Interface import Galaxea_R1_Interface

import pdb
import rosbag

ARM_TOPIC    = '/hdas/feedback_arm_right'       # 7 轴 JointState
HAND_TOPIC   = '/hdas/feedback_gripper_right'

def get_action(bagfile):
    """
    从 bag 中读取 ARM_TOPIC + HAND_TOPIC,
    每当 arm+hand 同时出现时，保存：(ts, action_list)
    其中 ts 是该帧相对 bag 起点的秒数，
    action_list 是 [7 arm angles + 6 hand angles]。
    """
    bag    = rosbag.Bag(bagfile, 'r')
    start_ts = bag.get_start_time()
    last_arm  = None    # 缓存临时的右臂命令
    last_hand = None    # 缓存临时的右爪反馈
    t_arm = t_hand = 0.0
    actions   = []      # 最终返回的动作列表

    for topic, msg, t in bag.read_messages(): #从rosbag读取动作帧
        rel_t = t.to_sec() - start_ts

        if  topic == ARM_TOPIC:
            # 读取 JointState.position里面的7轴关节反馈
            last_arm = list(msg.position)
            t_arm = rel_t
        elif topic == HAND_TOPIC:
            # JointState.position 就是 6 轴角度
            last_hand = list(msg.position)
            t_hand = rel_t
        # 如果 arm 和 hand 都已经读到，就组成一次完整“动作”,，取它们的max时间作为这一帧的时间
        if last_arm is not None and last_hand is not None:
            ts = max(t_arm, t_hand)
            actions.append((ts, last_arm + last_hand))
            # 清空，等待下一次 new arm+hand
            last_arm  = None
            last_hand = None
    bag.close()
    return actions

class RoboticsSystem:
    def __init__(self, args):
        self.args = args
        self.config_path = args.config_path
        self.config = self.load_inference_config()
        self.running = True
        self.height_threshold = 0

        # Create log file
        self.log_file_path = os.path.join(self.config['logging']['path'], 'log.txt')
        self.planner_log_file_path = os.path.join(self.config['logging']['path'], 'log_planner.txt')
        self.log_file = open(self.log_file_path, 'w')
        self.planner_log_file = open(self.planner_log_file_path, 'w')
        self.img_dir = os.path.join(self.config['logging']['path'], 'images')

        #For debug

        # self.head_image = Image.open("/data/yixiang/SRCB-DexVLA/croped_head_image.png")
        # self.head_image_np = np.asarray(self.head_image)

        self.actions = get_action("/data/GalaxeaDataset/20250430/None_20250430214414254_RAW.bag")

        self.in_post_run = True
    
        self.robot = Galaxea_R1_Interface(init=True)
        # self.head_image = self.robot.get_head_image()
        self.init_planner()
        print("init_planner done")
        self.init_controller()
        print("init_controller done")

        self.check_grasp_success_thread = threading.Thread(target=self.check_grasp_success, daemon=True)
        self.check_grasp_success_thread.start()
        print("init_threads done")
        

    def log(self, message):
        log(message, self.log_file)

    def load_inference_config(self):
        """Load system configuration from YAML file"""
        # config_path = os.path.join(os.path.dirname(__file__), 'inference_utils', 'config.yaml')
        with open(self.config_path, 'r') as f:
            return yaml.safe_load(f)
        
    def init_planner(self):
        self.planner = DexGraspVLAPlannerVL(
                model_path=self.config["planner"]["model_path_VL"])
            
        self.planner.set_logging(self.planner_log_file, self.img_dir)

    def init_controller(self):  
        self.device = torch.device('cuda:0')

        # main_config_path = os.path.join(os.path.dirname(__file__), 'controller', 'config', 'train_dexgraspvla_controller_workspace.yaml')
        # task_config_path = os.path.join(os.path.dirname(__file__), 'controller', 'config', 'task', 'grasp.yaml')
        
        # self.cfg = load_config(
        #     main_config_path=main_config_path,
        #     task_config_path=task_config_path
        # )
        # workspace = hydra.utils.get_class(self.cfg._target_)(self.cfg)
        # self.policy = workspace.model
        # self.policy.eval().to(self.device)

        # Initialize SAM
        sam_checkpoint = self.config['sam']['checkpoint']
        model_type = self.config['sam']['model_type']
        self.sam = sam_model_registry[model_type](checkpoint=sam_checkpoint)
        self.sam.to(device=self.device)
        self.predictor = SamPredictor(self.sam)
        
        # Initialize Cutie
        self.cutie = get_default_model()
        self.processor = InferenceCore(self.cutie, cfg=self.cutie.cfg)
        self.processor.max_internal_size = -1

    def check_grasp_success(self):
        while True:
            if not self.in_post_run:
                # current_pose = self.robot.right_arm.get_current_pose()
                # current_height = current_pose[2]
                current_height = 0.1
                if current_height > self.height_threshold:
                    # base64_image = encode_image_to_base64(self.third_color_image[..., ::-1])
                    # image_url = f"data:image/png;base64,{base64_image}"
                    head_image = self.robot.get_head_image(visulization=True)
                    with timer("check_grasp_success", self.log_file):
                        if self.planner.request_task(
                            image=self.conver_np_image_PIL(head_image),
                            # image = self.head_image,
                            task_name="check_grasp_success",
                        ):
                            self.in_post_run = True
                            self.log("Planner believes the task is done.")
            time.sleep(0.1)
            if not self.running:
                break

    def process_user_prompt(self):
        # Show head camera image
        # plt.imshow(self.head_image)
        # plt.axis('off')
        # plt.pause(1)

        # # Save head camera image
        # timestamp = time.strftime("%Y%m%d_%H%M%S")
        # img_filename = f"{timestamp}_head_image_for_user_prompt.png"
        # img_path = os.path.join(self.img_dir, img_filename)
        # cv2.imwrite(img_path, self.third_color_image)

        sys.stdin.flush()
        self.user_prompt = input("""Please enter your instruction: (It can be an abstract instruction like 'clear the table' or a specific object to be grabbed, such as 'grasp the red cups and blue cookies')\n>>>  """)
        self.log(f"User prompt: {self.user_prompt}")

        plt.close()

        with timer("classify user prompt", self.log_file):
            self.user_prompt_type = self.planner.request_task(
                            task_name="classify_user_prompt",
                            instruction=self.user_prompt,
                            max_token=256
            )
        self.log(f"User prompt type: {self.user_prompt_type}.")
        if self.user_prompt_type == "TypeI":  # Explicitly specifies grabbing specific items
            head_image = self.robot.get_head_image()
            with timer("decompose user prompt", self.log_file):
                self.object_list = self.planner.request_task(
                            image=self.conver_np_image_PIL(head_image),
                            # image = self.head_image,
                            task_name="decompose_user_prompt",
                            instruction=self.user_prompt,
                            max_token=512
                        )
                self.log(f"Object list: {self.object_list}.")
        elif self.user_prompt_type == "TypeII":  # Abstract instruction without specific details
            pass
        else:
            raise ValueError(f"The user prompt type {self.user_prompt_type} is invalid.")
    
    def get_current_instruction(self):
        if self.user_prompt_type == "TypeI":  # Explicitly specifies grabbing specific items
            self.current_instruction = self.object_list[0]
        elif self.user_prompt_type == "TypeII":  # Abstract instruction without specific details
            head_image = self.robot.get_head_image()
            with timer("generate instruction", self.log_file):
                self.current_instruction = self.planner.request_task(
                            task_name="generate_instruction",
                            image = head_image,
                            instruction=None,
                            max_token=512
                        )
        self.log(f"Current instruction: {self.current_instruction}")   

    def show_and_save_image_with_mask(self, image, mask, filename = None):
        """Save and display image with mask
        
        Args:
            image: Original image
            mask: Binary mask
            filename: Filename to save
        """
        # Create an image with the same size as the original
        height, width = image.shape[:2]
        fig = plt.figure(figsize=(width/100, height/100), dpi=100)
        ax = plt.Axes(fig, [0., 0., 1., 1.])  # Create axes without margins
        ax.set_axis_off()
        fig.add_axes(ax)
        
        # Display image
        ax.imshow(image)
        
        # Get mask color and random color settings from config
        mask_color = self.config['visualization']['mask']['color'][0]
        
        # Display mask with configured color
        show_mask(mask, ax, color=mask_color)
        
        # Save image
        if filename is not None:
            img_path = os.path.join(self.img_dir, filename)
            plt.savefig(img_path, bbox_inches='tight', pad_inches=0)
        
        # Display image
        plt.draw()
        plt.pause(0.5)
        plt.close(fig)
        
        self.log(f"Head camera image with mask saved.")


    def show_and_save_image_with_bbox(self, image, bbox, filename = None):
        """Save and display image with bounding box
        
        Args:
            image: Original image
            bbox: Bounding box coordinates [x1, y1, x2, y2]
            filename: Filename to save
        """
        # Create an image with the same size as the original
        # pdb.set_trace()
        height, width = image.shape[:2]
        fig = plt.figure(figsize=(width/100, height/100), dpi=100)
        ax = plt.Axes(fig, [0., 0., 1., 1.])  # Create axes without margins
        ax.set_axis_off()
        fig.add_axes(ax)
        
        # Display image
        ax.imshow(image)
        
        # Get bounding box color and line width from config
        bbox_color = self.config['visualization']['bbox']['color']
        bbox_linewidth = self.config['visualization']['bbox']['linewidth']
        
        # Draw bounding box
        x1, y1, x2, y2 = bbox.astype(int)
        rect = plt.Rectangle((x1, y1), x2-x1, y2-y1, 
                            linewidth=bbox_linewidth, 
                            edgecolor=bbox_color, 
                            facecolor='none')
        ax.add_patch(rect)
        
        # Save image
        if filename is not None:
            img_path = os.path.join(self.img_dir, filename)
            plt.savefig(img_path, bbox_inches='tight', pad_inches=0)
        
        # Display image
        plt.draw()
        plt.pause(0.5)
        plt.close(fig)
        
        self.log(f"Head camera image with bounding box saved.")

    def initialize_sam_cutie(self, bbox):
        self.processor.clear_memory()
        torch.cuda.empty_cache()
        head_image = self.robot.get_head_image()
        # head_image = self.head_image
        self.predictor.set_image(head_image)

        masks, scores, _ = self.predictor.predict(box=bbox, multimask_output=True)
        self.best_mask = masks[np.argmax(scores)]
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        img_with_mask_filename = f"{timestamp}_head_image_with_mask.png"
        # pdb.set_trace()
        # self.show_and_save_image_with_mask(self.head_image_np, self.best_mask, img_with_mask_filename)
        self.show_and_save_image_with_mask(head_image, self.best_mask, img_with_mask_filename)

        # Reinitialize Cutie
        self.mask = torch.from_numpy(self.best_mask.astype('uint8')).cuda()
        self.objects = np.unique(self.best_mask.astype('uint8'))
        self.objects = self.objects[self.objects != 0].tolist()
        self.cutie_initialized = False  # Reset Cutie initialization flag

    def reset_flags(self):
        # self.init_low_level_control = True
        # self.target_right_arm_joint = np.array(self.right_init_qpos)[None, :]
        # self.update_target = True
        # self.episode_start = time.time()
        self.in_post_run = False
        
    def check_complete(self):
        head_image = self.robot.get_head_image()
        with timer("check instruction complete", self.log_file):
            self.current_instruction_complete = self.planner.request_task(
                            image=self.conver_np_image_PIL(head_image),
                            # image = self.head_image,
                            task_name="check_instruction_complete",
                            instruction=self.current_instruction
                        )
        if self.current_instruction_complete:
            self.log(f"Current instruction <{self.current_instruction}> is completed.")
            if self.user_prompt_type == "TypeI":
                self.object_list = self.object_list[1:]
                user_prompt_complete = len(self.object_list) == 0
            else:
                with timer("check user prompt complete", self.log_file):
                    user_prompt_complete = self.planner.request_task(
                                    frame_path=head_image,
                                    task_name="check_user_prompt_complete",
                                    instruction=None
                                )
        else:
            self.log(f"Current instruction <{self.current_instruction}> is not completed.")
            user_prompt_complete = False
        if user_prompt_complete:
            self.log(f"User prompt <{self.user_prompt}> is completed.")
        else:
            self.log(f"User prompt <{self.user_prompt}> is not completed.")
        return user_prompt_complete

    def right_finish_reset(self):
        self.robot.init_hand()
        self.robot.init_arm()
        # self.move_to_target_joint_angle(self.right_placement_joint, step=0.05)
        # self.set_left_hand_open()
        # self.set_right_hand_open()
        # self.move_to_target_joint_angle(self.right_return_medium_joint, step=0.05)
        # self.move_to_target_joint_angle(self.right_init_qpos, step=0.05)

    def run_planner(self):
        while True:
            if not get_start_command():
                print("Bye.")
                break
            # Setup
            self.time_step = 0
            # self.init_episode(manual=False)
            self.process_user_prompt()
            
            while True:
                i = 0
                self.get_current_instruction()
                bbox = self.mark_bbox_planner()
                self.initialize_sam_cutie(bbox)
                self.reset_flags()
                self.log("Controller starts executing the current instruction.")
                # Execute grasping
                while True:
                    if self.in_post_run or i >= len(self.actions):
                        # pdb.set_trace()
                        break
                    if i % 5 == 0:
                        action = self.actions[i][1]
                        self.robot.move_step(action)
                    # state = self.robot.get_states() state: {'head_image', 'hand_image', 'arm_joint_positions' 14 right[7:14], 'hand_joint_positions' 6}
                    # mask = self.get_mask()
                    # obs = self.get_obs(state, mask)  # TODO: There might be misalignment between mask and image, needs verification
                    # attn_map_output_path = os.path.join(self.current_attn_maps_dir, f'{self.time_step}.pkl') if self.args.gen_attn_map else None
                    # self.time_step += 1
                    # obs_dict_np = self.process_obs(env_obs=obs, shape_meta=self.cfg.task.shape_meta)
                    # obs_dict = dict_apply(obs_dict_np, 
                    #     lambda x: torch.from_numpy(x).unsqueeze(0).to(self.device))
                    # with torch.no_grad():
                    #     action_pred = self.policy.predict_action(obs_dict, attn_map_output_path)
                    #     action = action_pred[0].detach().to('cpu').numpy()

                    # self.robot.move_step(action)
                    i += 1
                self.right_finish_reset()
                user_prompt_complete = self.check_complete()
                if user_prompt_complete:
                    break
            # self.close_episode()

    def conver_np_image_PIL(self, numpy_image):
        return Image.fromarray(np.uint8(numpy_image)).convert('RGB')

    def mark_bbox_planner(self):
        head_image = self.robot.get_head_image()
        # head_image = self.head_image
        with timer("mark bounding box", self.log_file):
            bbox_info = self.planner.request_task(
                    image=self.conver_np_image_PIL(head_image),
                    # image = head_image,
                    task_name="mark_bounding_box",
                    instruction=self.current_instruction
                )
        bbox = bbox_info['bbox_2d']
        self.log(f"Bounding box marked by the planner: {bbox}.")
        bbox = np.array(bbox)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        img_with_bbox_filename = f"{timestamp}_head_image_with_bbox.png"
        self.show_and_save_image_with_bbox(head_image, bbox, img_with_bbox_filename)
        # self.show_and_save_image_with_bbox(self.head_image_np, bbox, img_with_bbox_filename)
        return bbox
    
    def close(self):
        try:
            # First set flag to stop all threads
            self.running = False
            
            # Wait for all threads to end
            if hasattr(self, 'check_grasp_success_thread'):
                self.check_grasp_success_thread.join(timeout=2)

        except Exception as e:
            print(f"Error during close: {str(e)}")
        finally:
            print("System shutdown complete.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='DexGraspVLA Inference')
    parser.add_argument('--manual', action='store_true', help='Manually select the target object.')
    parser.add_argument('--config_path', type=str, default="config.yaml", help='Manually select the target object.')
    parser.add_argument('--use_omni', action='store_true', help='Manually select the target object.')
    args = parser.parse_args()
    system = RoboticsSystem(args)
    system.run_planner()
    # robot = Galaxea_R1_Interface()
    # robot_planner = Planner(args)

    # task_name = "decompose_user_prompt"
    # image = Image.open("case1_Color.png")
    # instruction = "clear table"

    # object_list = robot_planner.planner.request_task(task_name,image,instruction)
    # print(object_list)
    # for object in object_list:
    #     task_name = "mark_bounding_box"
    #     instruction = object
    #     robot_planner.planner.request_task(task_name,image,instruction)