import os
import time
import cv2
import torch
import numpy as np
import matplotlib.pyplot as plt
from segment_anything import SamPredictor, sam_model_registry
from inference_utils.utils import show_mask
from cutie.inference.inference_core import InferenceCore
from cutie.utils.get_default_model import get_default_model
from torchvision.transforms.functional import to_tensor
import yaml
import zarr
from datetime import datetime

class SimpleRoboticsSystem:
    def __init__(self):
        self.img_dir = "/data/dingzher/DexGrasp_Demo/SRCB-DexVLA/temp_1"
        self.third_color_image = cv2.imread('/data/dingzher/DexGrasp_Demo/SRCB-DexVLA/temp/head_cam.png', cv2.IMREAD_COLOR)
        self.end_color_image = cv2.imread('/data/dingzher/DexGrasp_Demo/SRCB-DexVLA/temp/head_cam.png', cv2.IMREAD_COLOR)
        self.device = torch.device('cuda:0')
        self.config = self.load_inference_config()
        # Initialize SAM
        self.sam_checkpoint = '/data/dingzher/DexGrasp_Demo/SRCB-DexVLA/checkpoint/sam/sam_vit_h_4b8939.pth'
        self.model_type = 'vit_h'  # Example: Adjust based on your SAM model type
        self.sam = sam_model_registry[self.model_type](checkpoint=self.sam_checkpoint)
        self.sam.to(device=self.device)
        self.predictor = SamPredictor(self.sam)

        # Initialize Cutie
        self.cutie = get_default_model()
        self.processor = InferenceCore(self.cutie, cfg=self.cutie.cfg)
        self.processor.max_internal_size = -1
        
        # Initialize mask variables
        self.mask = None
        self.objects = []
        self.cutie_initialized = False
    
    def load_inference_config(self):
        """Load system configuration from YAML file"""
        config_path = os.path.join(os.path.dirname(__file__), 'DexGraspVLA', 'inference_utils', 'config.yaml')
        with open(config_path, 'r') as f:
            return yaml.safe_load(f)
    
    def show_and_save_image_with_bbox(self, image, bbox, filename):
        """Save and display image with bounding box
        
        Args:
            image: Original image
            bbox: Bounding box coordinates [x1, y1, x2, y2]
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
        img_path = os.path.join(self.img_dir, filename)
        plt.savefig(img_path, bbox_inches='tight', pad_inches=0)
        
        # Display image
        plt.draw()
        plt.pause(0.5)
        plt.close(fig)
        
        print(f"Head camera image with bounding box saved.")
        
    def mark_bbox_manual(self):
        """Manually mark bounding box on the image."""
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        img_filename = f"{timestamp}_head_image_start.png"
        img_path = os.path.join(self.img_dir, img_filename)
        cv2.imwrite(img_path, self.third_color_image)
        img_filename_end = f"{timestamp}_head_image_end.png"
        img_path_end = os.path.join(self.img_dir, img_filename_end)
        cv2.imwrite(img_path_end,  self.end_color_image)        
        
        # Display image and get bounding box
        plt.figure()
        plt.imshow(self.third_color_image[..., ::-1])  # Convert BGR to RGB for display
        plt.axis('off')
        plt.title("Please click two points to define the bounding box (top left and bottom right)")
        bbox_points = plt.ginput(n=2, timeout=0)
        plt.close()
        (x1, y1), (x2, y2) = bbox_points
        bbox = np.array([min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2)])
        
        # Save image with bounding box
        img_with_bbox_filename = f"{timestamp}_head_image_with_bbox.png"
        self.show_and_save_image_with_bbox(self.third_color_image[..., ::-1], bbox, img_with_bbox_filename)
        return bbox
    
    def initialize_sam_cutie(self, bbox):
        """Initialize SAM and generate the mask for the image."""
        self.predictor.set_image(self.third_color_image[..., ::-1])
        masks, scores, _ = self.predictor.predict(box=bbox, multimask_output=True)
        best_mask = masks[np.argmax(scores)]
        
        # Save the image with initial mask
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        img_with_mask_filename = f"{timestamp}_head_image_with_mask.png"
        self.show_and_save_image_with_mask(self.third_color_image[..., ::-1], best_mask, img_with_mask_filename)
        
        # Initialize Cutie for tracking
        self.mask = torch.from_numpy(best_mask.astype('uint8')).cuda()
        self.objects = np.unique(self.mask.cpu().numpy())
        self.objects = self.objects[self.objects != 0].tolist()
        self.cutie_initialized = False  # Set Cutie initialized flag to False

    def get_mask(self):
        """Update Cutie mask based on the current image."""
        with torch.no_grad():
            # Convert image to tensor
            image_tensor = to_tensor(self.third_color_image[..., ::-1].copy()).cuda().float()
            
            # Update the Cutie mask
            if not self.cutie_initialized:
                output_prob = self.processor.step(image_tensor, self.mask, objects=self.objects)
                self.cutie_initialized = True
            else:
                output_prob = self.processor.step(image_tensor)
                
            # Convert the output to a binary mask
            current_mask = self.processor.output_prob_to_mask(output_prob)
            current_mask_np = current_mask.cpu().numpy().astype(np.uint8)
        
        return current_mask_np
    
    def show_and_save_image_with_mask(self, image, mask, filename):
        """Display and save image with mask."""
        height, width = image.shape[:2]
        fig = plt.figure(figsize=(width / 100, height / 100), dpi=100)
        ax = plt.Axes(fig, [0., 0., 1., 1.])  # Create axes without margins
        ax.set_axis_off()
        fig.add_axes(ax)

        # Display original image
        ax.imshow(image)

        # Display mask with configured color
        mask_color = [0.1, 0.5, 1.0, 0.6]  # Example: semi-transparent blue mask
        show_mask(mask, ax, color=mask_color)
        
        if filename is not None:
            img_path = os.path.join(self.img_dir, filename)
            plt.savefig(img_path, bbox_inches='tight', pad_inches=0)

        # Display image
        plt.draw()
        plt.pause(0.5)
        plt.close(fig)

    def run(self):
        """Run the manual bounding box marking, SAM mask initialization, and Cutie tracking."""
        input_path = "/data/dingzher/DexGrasp_Demo/SRCB-DexVLA/zarr_data_transfer/output_data_20250529_single_bottle.zarr"
        f = zarr.open(input_path)
        right_cam_img_data = f['data/rgbm'][:]
        episode_ends = f['meta/episode_ends'][:]
        episode_ends = np.insert(episode_ends, 0,0)
        rgbm_with_mask = []
        timestamp = datetime.now().strftime("%Y%m%d-%H%M")
        file_name = f"/data/dingzher/DexGrasp_Demo/SRCB-DexVLA/zarr_data_transfer/output_{timestamp}_array.txt"
        
        for i in range(len(episode_ends) - 1):
            self.third_color_image = right_cam_img_data[episode_ends[i]]
            self.end_color_image = right_cam_img_data[episode_ends[i+1] - 100]
            while True:
                bbox = self.mark_bbox_manual()
                self.initialize_sam_cutie(bbox)
                user_input = input("Press 'c' to exit or any other key to continue: ").strip().lower()
                if user_input == 'c':
                    with open(file_name, "a") as file:
                        file.write("\n" + "light green bowl" + ":  " + str(bbox))
                    break
            # Track mask over time (or iterations)
            for j in range(episode_ends[i], episode_ends[i+1]):  # Example: Track for 10 frames
                # Simulate new image capture (you could replace this with actual frame updates)
                self.third_color_image = right_cam_img_data[j]
                # Get updated mask
                current_mask = self.get_mask()
                current_mask_tmp = current_mask.astype(np.uint8)[..., np.newaxis]
                rgbm_frame = np.concatenate([self.third_color_image, current_mask_tmp.astype(np.uint8)], axis=-1)
                rgbm_with_mask.append(rgbm_frame)
                # self.show_and_save_image_with_mask(self.third_color_image[..., ::-1], current_mask, f"{i}-{j}.png")
        rgbm_with_mask = np.stack(rgbm_with_mask, axis=0)
        f['data'].create_dataset('rgbm', data=rgbm_with_mask, overwrite=True)
        print("Compeleted.")
if __name__ == "__main__":
    system = SimpleRoboticsSystem()
    system.run()
