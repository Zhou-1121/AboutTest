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

from DexGraspVLA.planner.dexgraspvla_planner_v2 import DexGraspVLAPlanner
from DexGraspVLA.inference_utils.audio_processor import InputThread

class RoboticsSystem:
    def __init__(self, args):
        self.args = args
        self.config_path = args.config_path
        self.config = self.load_inference_config()
        
        self.audio_thread = InputThread(device_name="sysdefault")
        self.audio_thread.daemon = True
        
        self.running = True

        self.init_planner()
        
        self.audio_thread.start()
        print("init_planner done")

    def load_inference_config(self):
        """Load system configuration from YAML file"""
        # config_path = os.path.join(os.path.dirname(__file__), 'inference_utils', 'config.yaml')
        with open(self.config_path, 'r') as f:
            return yaml.safe_load(f)
        
    def init_planner(self):
        self.planner = DexGraspVLAPlanner(
            model_path=self.config["planner"]["model_path"],model_type="qwen_omni",enable_stream=True)
        
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='DexGraspVLA Inference')
    parser.add_argument('--manual', action='store_true', help='Manually select the target object.')
    parser.add_argument('--config_path', type=str, default="/data/shiqi/SRCB-DexGraspVLA-Project/config.yaml", help='Manually select the target object.')
    args = parser.parse_args()
    system = RoboticsSystem(args)
    system.load_inference_config()
    import pdb
    # pdb.set_trace()

    while True:
        audio_instruction_path = system.audio_thread.wait_input() # user input interface
        # instruction = "clear table"
        # object_list = system.planner.request_task(task_name,frame_path="/data/yixiang/SRCB-DexVLA/temp/case1_Color.png",instruction=None,instruction_audio_path = audio_instruction_path)
        # pdb.set_trace()
        # print(object_list)
        instructions = [
            'clean the table_zh.wav',
            'clean the table.wav',
            'what_is_on_the_disk_zh.wav'
        ]
        image = Image.open("croped_head_image.png")
        
        task = 'classify_user_prompt_zero' # response_with_audio
        for ins in instructions:
            isaction, reason = system.planner.request_task(task,frame_path=image,instruction=None,instruction_audio_path = ins)

            if isaction:
                print(f"action prompt: {reason}")
                object_list = system.planner.request_task('decompose_user_prompt',frame_path=image,instruction=reason, audio_instruction_path = None)
                print(f"object_list: {object_list}")
                for object in object_list:
                    task_name = "mark_bounding_box"
                    instruction = object
                    system.planner.request_task(task_name,image,instruction)

            else:
                print("QA prompt: {reason}")
                response_text = system.planner.request_task('response_with_audio',frame_path=image,instruction=reason)
                print(f"response: {response_text}")

    