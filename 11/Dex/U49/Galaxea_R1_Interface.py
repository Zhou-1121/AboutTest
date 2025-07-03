#!/usr/bin/env python

import rospy
import yaml
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import JointState, Image
from geometry_msgs.msg import PoseStamped
from inspire_hand_modbus.srv import (
    set_angle, set_angleRequest,
    get_angle_act, get_angle_actRequest, 
    set_speed, set_speedRequest
)
import time
from cv_bridge import CvBridge
import cv2
import tf
import numpy as np
import torch
import matplotlib.pyplot as plt
from DexGraspVLA.inference_utils.utils import show_mask
class Galaxea_R1_Interface():
    def __init__(self, init = False):
        #在这部分检查是否已经完成ROS节点初始化、
        if not rospy.core.is_initialized():
            rospy.init_node('Galaxea_R1_Interface') # 初始化ROS节点
        self.config = self.load_inference_config("config.yaml")
    
        self.arm_config = self.config['robot']['arms']
        self.hand_config = self.config['robot']['hands']

        self.left_joint_state_pub = rospy.Publisher(self.arm_config['left']['move_topic'], JointState, queue_size=1,latch=True)
        self.right_joint_state_pub = rospy.Publisher(self.arm_config['right']['move_topic'], JointState, queue_size=1, latch=True)
        print(self.config['robot']['torso']['move_topic'])
        self.left_arm_ee_pub = rospy.Publisher(self.arm_config['left']['move_ee_topic'], PoseStamped, queue_size=1,latch=True)
        self.right_arm_ee_pub = rospy.Publisher(self.arm_config['right']['move_ee_topic'], PoseStamped, queue_size=1,latch=True)
        self.torso_joint_state_pub_real = rospy.Publisher(self.config['robot']['torso']['move_topic'], JointState, queue_size=1,latch=True)
        self.bridge = CvBridge()
        rospy.sleep(1.0)
        self.rate = rospy.Rate(50)
        # self.right_hand = Inspire_Hand_Interface(self.hand_config['right']['read_topic'], self.hand_config['right']['move_topic'], self.hand_config['right']['default_speed'])
        if init:
            self.init_robot()
        print("Galaxea R1 Pro Initilization Finish!!")
        # get_angle_topic = self.config['robot']['hands']['right']['read_topic']
        # move_hand_service = self.config['robot']['hands']['right']['move_topic']
        # hand_speed = self.config['robot']['hands']['right']['default_speed']
        # self.right_hand_interface = Inspire_Hand_Interface(get_angle_topic, move_hand_service, hand_speed)
        print("Hand Initilization Finish!!")
        self.states = {"head_image": None,
                      "left_hand_image": None,
                      "right_hand_image": None,
                      "arm_joint_positions": None, # left + right
                      "hand_joint_positions": None}
        
        self.device = torch.device('cuda:0')
    def init_robot(self):
        self.init_hand()
        self.init_arm()   
        self.move_torso(self.config['robot']['torso']['init_qpos'])
        time.sleep(2)
        
    def init_arm(self):
        # self.move_right_arm(self.arm_config['right']['init_qpos'])
        self.move_arm(self.arm_config['left']['init_qpos'], self.arm_config['right']['init_qpos'])
        time.sleep(2)
    
    def init_hand(self):
        self.right_hand.move_hand(self.hand_config["right"]["default_standby"])
        time.sleep(3)
        
    def get_arm_position(self):
        left_arm_joint_msg = rospy.wait_for_message(self.arm_config['left']['read_topic'], JointState, timeout=5)
        right_arm_joint_msg = rospy.wait_for_message(self.arm_config['right']['read_topic'], JointState, timeout=5)
        arm_position = [list(left_arm_joint_msg.position), list(right_arm_joint_msg.position)]
        return arm_position
    
    def _crop_bottom_center(self, img, crop_w=640, crop_h=480):
        """
        从 img 底部居中裁剪 crop_w x crop_h
        """
        h, w = img.shape[:2]
        if crop_w > w or crop_h > h:
            raise ValueError(f"裁剪区域({crop_w}x{crop_h})大于原图({w}x{h})")
        start_x = (w - crop_w) // 2
        start_y = h - crop_h
        # start_y = (h - crop_h) // 2   #中心居中
        return img[start_y:start_y+crop_h, start_x:start_x+crop_w]

    def get_head_image(self, visulization = False):
        hand_camera_msg = rospy.wait_for_message(self.config['cameras']['head_cameras']['topic'], Image, timeout=5)
        cv_image = self.bridge.imgmsg_to_cv2(hand_camera_msg, desired_encoding='bgr8')
        cv_image = self._crop_bottom_center(cv_image, 640, 480)
        # cv2.imwrite("croped_head_image.png",cv_image)
        if visulization:
            print(cv_image.shape)
            cv2.imshow('head camera image', cv_image)
            cv2.waitKey(0)
            cv2.destroyAllWindows()            
        return cv_image[...,::-1] # to rgb 
    
    def get_hand_image(self, right_hand = True, visulization = False):
        print(self.config['cameras']['right_hand_cameras']['topic'])
        if right_hand:
            hand_camera_msg = rospy.wait_for_message(self.config['cameras']['right_hand_cameras']['topic'], Image, timeout=5)
        else:
            hand_camera_msg = rospy.wait_for_message(self.config['cameras']['left_hand_cameras']['topic'], Image, timeout=5)
            
        cv_image = self.bridge.imgmsg_to_cv2(hand_camera_msg, desired_encoding='bgr8')
        
        if visulization:
            print(cv_image.shape)
            cv2.imshow('head camera image', cv_image)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            
        return cv_image[...,::-1] # to rgb 
    
    def move_arm(self, position_left, position_right):
        left_joint_state = JointState()
        left_joint_state.position = position_left
        left_joint_state.velocity = self.arm_config['left']['default_qvel']
        right_joint_state = JointState()
        right_joint_state.position = position_right
        right_joint_state.velocity = self.arm_config['right']['default_qvel']
        left_joint_state.header.stamp = rospy.Time.now()
        self.left_joint_state_pub.publish(left_joint_state)
        self.rate.sleep()
        right_joint_state.header.stamp = rospy.Time.now() 
        self.right_joint_state_pub.publish(right_joint_state)
        self.rate.sleep()

    def move_right_arm(self, position_right):
        right_joint_state = JointState()
        right_joint_state.position = position_right
        right_joint_state.velocity = self.arm_config['right']['default_qvel']

        self.right_joint_state_pub.publish(right_joint_state)
        self.rate.sleep()

    def move_torso(self, position_torso):
        torso_joint_state = JointState()
        torso_joint_state.position = position_torso
        torso_joint_state.velocity = self.config['robot']['torso']['default_qvel']
        
        self.torso_joint_state_pub_real.publish(torso_joint_state) 
        self.rate.sleep()
        print("Moving torso")
        
    def move_to_zero_postion(self):
        self.move_torso([0,0,0,0])
        time.sleep(3)
        self.move_arm([0,0,0,0,0,0,0],[0,0,0,0,0,0,0])
        # time.sleep(2)
        # self.right_hand.move_hand(self.hand_config["right"]["default_open"])
    
    def move_arm_to_init(self):
        self.move_arm(self.arm_config['left']['init_qpos'], self.arm_config['right']['init_qpos'])
    
    def load_inference_config(self, config_path):
        """Load system configuration from YAML file"""
        # config_path = os.path.join(os.path.dirname(__file__), 'inference_utils', 'config.yaml')
        with open(config_path, 'r') as f:
            return yaml.safe_load(f)
        
    
    def get_cam_image(self):
        pass
    # def hand_init(self,):
        # self.pub_hand_position = rospy.Publisher('/hand/set_cmd', Int32MultiArray, queue_size = 1)
    #     self.rate = rospy.Rate(10)
    #     self.pub_hand_position = rospy.Publisher('topic_name', topic_messager, queue_size = 1)

    def get_states(self):
        
        self.states['head_image'] = self.get_head_image()
        self.states['hand_image'] = self.get_hand_image()
        self.states['arm_joint_positions'] = self.get_arm_position()
        self.states['hand_joint_positions'] = self.right_hand.get_hand_position()
        return self.states
    
    def move_step(self, action):
        self.right_hand.move_hand(action[7:13])
        self.move_right_arm(action[0:7])
        
    def place_object(self):
        self.move_right_arm(self.arm_config['right']['placement_joint'])
        time.sleep(2)
        self.right_hand.move_hand(self.hand_config["right"]["default_standby"])
        time.sleep(1)
        
    def move_ee(self, position, orientation, euler_mode=False, right_arm=True):
        target_pose = PoseStamped()

        target_pose.pose.position.x = position[0]
        target_pose.pose.position.y = position[1]
        target_pose.pose.position.z = position[2]

        if euler_mode:
            # 欧拉角转四元数 (roll, pitch, yaw)
            quaternion = tf.transformations.quaternion_from_euler(
                orientation[0], orientation[1], orientation[2]
            )
            target_pose.pose.orientation.x = quaternion[0]
            target_pose.pose.orientation.y = quaternion[1]
            target_pose.pose.orientation.z = quaternion[2]
            target_pose.pose.orientation.w = quaternion[3]
        else:
            target_pose.pose.orientation.x = orientation[0]
            target_pose.pose.orientation.y = orientation[1]
            target_pose.pose.orientation.z = orientation[2]
            target_pose.pose.orientation.w = orientation[3]

        for _ in range(2):
            if right_arm:
                target_pose.header.stamp = rospy.Time.now()
                self.right_arm_ee_pub.publish(target_pose)
                self.rate.sleep()
            else:
                target_pose.header.stamp = rospy.Time.now()
                self.left_arm_ee_pub.publish(target_pose)
                self.rate.sleep()

    def get_segment(self, mode = "manual", object_name = None, visulizaion=False):
        print(">>> Start Segementaion Program")
        from segment_anything import sam_model_registry, SamPredictor
        image = self.get_head_image()
        sam_checkpoint = "/data/yixiang/SRCB-DexVLA/checkpoint/sam/sam_vit_h_4b8939.pth"
        sam = sam_model_registry["vit_h"](checkpoint=sam_checkpoint)
        sam.to(device=self.device)
        predictor = SamPredictor(sam)
        print(">>> Load SAM Finish")
        if mode == "manual":
            # Display image and get bounding box
            plt.figure()
            plt.imshow(image)  # The image is BGR
            plt.axis('off')
            plt.title("Please click two points to define the bounding box (top left and bottom right)")
            bbox_points = plt.ginput(n=2, timeout=0)
            plt.close()
            (x1, y1), (x2, y2) = bbox_points
            bbox = np.array([min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2)])
        elif mode == "auto":
            from DexGraspVLA.planner.qwen2_5_vl_7b_robot_planner import DexGraspVLAPlanner as DexGraspVLAPlannerVL
            planner = DexGraspVLAPlannerVL("/data/model/Qwen/Qwen2.5-VL-7B")
            instruction = f"get BBox for object {object_name} on the table"
            from PIL import Image
            PIL_image = Image.fromarray(image).convert('RGB')
            bbox_info = planner.request_task("mark_bounding_box",PIL_image, instruction)
            bbox = bbox_info['bbox_2d']
            bbox = np.array(bbox)
        else:
            raise ValueError(f"Invalid mode: {mode}")
        
        predictor.set_image(image)
        masks, scores, _ = predictor.predict(box=bbox, multimask_output=True)
        best_mask = masks[np.argmax(scores)]
        if visulizaion:
            self.show_and_save_image_with_mask(image, bbox, best_mask, object_name, "test_image.png")
        return best_mask
    
    def show_and_save_image_with_mask(self, image, bbox, mask, class_name, image_path, i = 0):
        """Save and display image with mask
        
        Args:
            image: Original image
            mask: Binary mask
            image_path: Filename to save
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
        mask_color = self.config['visualization']['mask']['color'][i]
        
        index = np.argwhere(mask == 1)
        x1, y1, x2, y2 = np.min(index[:,1]), np.min(index[:,0]), np.max(index[:,1]), np.max(index[:,0])
        # x1, y1, x2, y2 = bbox.astype(int)
        rect = plt.Rectangle((x1, y1), x2-x1, y2-y1, 
                            linewidth=2, 
                            edgecolor=mask_color, 
                            facecolor='none')
        ax.add_patch(rect)
        ax.text( x1 + 5, y1 - 5, class_name, color= mask_color, fontsize=16, verticalalignment='bottom', horizontalalignment='left')
        # Display mask with configured color
        show_mask(mask, ax, color=mask_color)

        plt.savefig(image_path, bbox_inches='tight', pad_inches=0, format = "jpg")
        
        plt.show()
        plt.close(fig)
class Inspire_Hand_Interface():
    def __init__(self, read_topic, write_topic, speed_list):
        self.get_hand_angle_topic = read_topic
        self.move_hand_topic = write_topic
        self.CMD_TYPE = ""
        self.HAND_ID = 1
        self.set_speed(speed_list)

    def set_speed(self, speed_list):
        """
        调用 /inspire_hand_modbus/set_speed 来设置六轴速度
        """
        if len(speed_list) != 6:
            rospy.logerr("速度列表长度必须为6")
            return False

        rospy.loginfo(f"设置速度：{speed_list}")
        try:
            rospy.wait_for_service('/inspire_hand_modbus/set_speed')
            proxy = rospy.ServiceProxy('/inspire_hand_modbus/set_speed', set_speed)
            req = set_speedRequest()
            # 如果有 status/id 字段也一并设置，这里假设仅 speedX
            req.speed0, req.speed1, req.speed2, req.speed3, req.speed4, req.speed5 = speed_list
            resp = proxy(req)
            rospy.loginfo(f"set_speed 返回: {resp}") # 看 set_speed.srv 的 response 字段
            return True
        except rospy.ServiceException as e:
            rospy.logerr(f"调用 set_speed 服务失败: {e}")
            return False
        
    def get_hand_position(self):
        try:
            rospy.wait_for_service(self.get_hand_angle_topic)
            proxy = rospy.ServiceProxy(self.get_hand_angle_topic, get_angle_act)
            req = get_angle_actRequest()
            req.status = self.CMD_TYPE
            req.id  = self.HAND_ID
            resp = proxy(req)
            return list(resp.curangle)
        
        except rospy.ServiceException as e:
            rospy.logerr(f"get_angle_act 服务调用失败: {e}")
    
    def move_hand(self, target=[0, 0, 0, 0, 0, 0]):
        try:
            target = [(int(x)) for x in target]
            rospy.wait_for_service(self.move_hand_topic)
            proxy = rospy.ServiceProxy(self.move_hand_topic, set_angle)
            req = set_angleRequest()
            req.status = "" # 替换实际的命令类型
            req.id = 1 # 替换为实际的ID（灵巧手ID）
            req.angle0, req.angle1, req.angle2, req.angle3, req.angle4, req.angle5 = target
            resp = proxy(req)
            rospy.loginfo(f"设置灵巧手角度：{target} → accepted={resp.angle_accepted}")
        except rospy.ServiceException as e:
            rospy.logerr(f"调用 set_angle 服务失败: {e}")
    #     self.get_hand_position()
    
    # def move_step(self, target = [0,0,0,0,0,0]):
    #     self.publish_hand_position(target)

    # def get_hand_position(self):
    #     msg = rospy.wait_for_message("topic_name", topic_messager)
    #     return msg
    
    # def publish_hand_position(self, target):
    #     msg = JointState()
    #     msg.name = "exampe"
    #     msg.header.stamp = rospy.Time.now() 
    #     msg.position = target
    #     self.pub_hand_position.publish(msg)

if __name__ == "__main__":
    robot = Galaxea_R1_Interface(init=False)
    # robot.move_arm_to_init()
    # time.sleep(3)
    # robot.get_segment(visulizaion=True)
    # robot.get_segment(mode = "auto", object_name= "red box", visulizaion=True)
    # robot.move_arm([0,0,0,0,0,0,0],[0,0,0,0,0,0,0])
    # robot.move_ee(robot.arm_config['right']['init_ee'][0:3], robot.arm_config['right']['init_ee'][3:7])
    # robot.move_ee([0,-0.2521,-0.4016], [0, 0, 0, 1])
    # robot.move_ee(robot.arm_config['left']['init_ee'][0:3], robot.arm_config['left']['init_ee'][3:7], right_arm = False)
    # robot.place_object()
    # robot.right_hand.move_hand(target = robot.hand_config["right"]["default_grasp"])
    # print(hand_pos)
    # states = robot.get_states()
    # print("robot_states: ", states)
    # robot.right_hand.move_hand_step(robot.hand_config['right']['default_sctandby'])
    # image = robot.get_head_image(visulization=True)
    # image = robot.get_hand_image(visulization=True)
    # curr_arm_pos = robot.get_arm_position()
    # print("current_arm_pos: ", curr_arm_pos)
    # target = [0,0,0,0,0,0] #之前测试的结果
    # robot.move_step(target)
    # time.sleep(5)
    robot.move_to_zero_postion()
    # robot.move_torso([0.0,0,0,0])
# pose: 
#   position: 
#     x: -0.0015147614669436087
#     y: -0.25578511422527
#     z: -0.40166505517829243
#   orientation: 
#     x: 0.0007446103611281364
#     y: 0.0009475189250318133
#     z: -0.0012802576559744659
#     w: 0.9999984543506212

#   position: 
#     x: 0.526860581279949
#     y: -0.33628150861578526
#     z: 0.12191954658404046
#   orientation: 
#     x: -0.57850393035014
#     y: -0.45112511937298627
#     z: 0.6176437668138541
#     w: 0.2834351893398958