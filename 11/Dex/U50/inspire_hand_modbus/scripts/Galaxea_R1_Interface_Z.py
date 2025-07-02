#!/usr/bin/env python

import rospy
import yaml
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import JointState, Image
from inspire_hand_modbus.srv import (
    set_angle, set_angleRequest,
    get_angle_act, get_angle_actRequest, 
    set_speed, set_speedRequest
)
import time
from cv_bridge import CvBridge
import cv2

class Galaxea_R1_Interface():

    def __init__(self):
        #在这部分检查是否已经完成ROS节点初始化、
        if not rospy.core.is_initialized():
            rospy.init_node('Galaxea_R1_Interface') # 初始化ROS节点
        self.config = self.load_inference_config("/home/samsung/yixiang/config.yaml")
    
        self.arm_config = self.config['robot']['arms']
        self.hand_config = self.config['robot']['hands']

        self.left_joint_state_pub = rospy.Publisher(self.arm_config['left']['move_topic'], JointState, queue_size=10)
        self.right_joint_state_pub = rospy.Publisher(self.arm_config['right']['move_topic'], JointState, queue_size=10)
        print(self.config['robot']['torso']['move_topic'])
        self.torso_joint_state_pub_real = rospy.Publisher(self.config['robot']['torso']['move_topic'], JointState, queue_size=10)
        self.bridge = CvBridge()
        self.right_hand = Inspire_Hand_Interface(self.hand_config['right']['read_topic'], self.hand_config['right']['move_topic'], self.hand_config['right']['default_speed'])
        # self.init_robot()
        print("Galaxea R1 Pro Initilization Finish!!")
        # get_angle_topic = self.config['robot']['hands']['right']['read_topic']
        # move_hand_service = self.config['robot']['hands']['right']['move_topic']
        # hand_speed = self.config['robot']['hands']['right']['default_speed']
        # self.right_hand_interface = Inspire_Hand_Interface(get_angle_topic, move_hand_service, hand_speed)
        # print("Hand Initilization Finish!!")
        self.states = {"head_image": None,
                      "left_hand_image": None,
                      "right_hand_image": None,
                      "arm_joint_positions": None, # left + right
                      "hand_joint_positions": None}
        
        # # 订阅 VR Teleop 夹爪目标位置
        # self.left_gripper_sub = rospy.Subscriber(
        #     '/motion_target/target_position_gripper_left',
        #     JointState, self._on_left_gripper_cmd)
        self.right_gripper_sub = rospy.Subscriber(
            '/motion_target/target_position_gripper_right',
            JointState, self._on_right_gripper_cmd)
        print("test")
        # 用于记录夹爪状态（初始状态为开爪）
        self.last_left_gripper_pct = 100.0
        self.last_right_gripper_pct = 100.0
        
    def init_robot(self):
        self.move_arm(self.arm_config['left']['init_qpos'], self.arm_config['right']['init_qpos'])
        time.sleep(2)
        self.move_torso(self.config['robot']['torso']['init_qpos'])
        time.sleep(3)
        self.right_hand.move_hand(self.hand_config["right"]["default_standby"])
        time.sleep(3)
    
    def get_arm_position(self):
        left_arm_joint_msg = rospy.wait_for_message(self.arm_config['left']['read_topic'], JointState, timeout=5)
        right_arm_joint_msg = rospy.wait_for_message(self.arm_config['right']['read_topic'], JointState, timeout=5)
        arm_position = [list(left_arm_joint_msg.position), list(right_arm_joint_msg.position)]
        return arm_position
    
    def get_head_image(self, visulization = False):
        hand_camera_msg = rospy.wait_for_message(self.config['cameras']['hand_cameras']['topic'], Image, timeout=5)
        cv_image = self.bridge.imgmsg_to_cv2(hand_camera_msg, desired_encoding='bgr8')
        
        if visulization:
            print(cv_image.shape)
            cv2.imshow('head camera image', cv_image)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            
        return cv_image  
    
    def move_arm(self, position_left, position_right):
        left_joint_state = JointState()
        left_joint_state.position = position_left
        left_joint_state.velocity = self.arm_config['left']['default_qvel']
        right_joint_state = JointState()
        right_joint_state.position = position_right
        right_joint_state.velocity = self.arm_config['right']['default_qvel']

        for i in range(4): # Need figure-out reason
            self.left_joint_state_pub.publish(left_joint_state)
            self.right_joint_state_pub.publish(right_joint_state)
            time.sleep(0.2)
        # for i in range(2): # Need figure-out reason
        #     self.right_joint_state_pub.publish(right_joint_state)
        #     time.sleep(0.1)

    def move_torso(self, position_torso):

        torso_joint_state = JointState()
        torso_joint_state.position = position_torso
        torso_joint_state.velocity = self.config['robot']['torso']['default_qvel']
        print(torso_joint_state)
        for i in range(4): # Need figure-out reason
            self.torso_joint_state_pub_real.publish(torso_joint_state) 
            time.sleep(0.1)
        
    def move_to_zero_postion(self):
        self.move_torso([0,0,0,0])
        time.sleep(3)
        self.move_arm([0,0,0,0,0,0,0],[0,0,0,0,0,0,0])
        time.sleep(2)
        self.right_hand.move_hand(self.hand_config["right"]["default_open"])
    
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
        self.states['arm_joint_positions'] = self.get_arm_position()
        self.states['hand_joint_positions'] = self.right_hand.get_hand_position()

        return self.states

    # def _on_left_gripper_cmd(self, msg):
    #     """
    #     监听 VR Teleop 左夹爪目标位置,根据按下T键来控制灵巧手开合
    #     """
    #     pct = msg.position[0]
    #     if pct != self.last_left_gripper_pct:   # 检查是否发生变化
    #         rospy.loginfo(f"左夹爪状态变化：{pct}%")
    #         # 根据百分比（按键触发）决定是开合
    #         if pct > 50: # 如果大于50，灵巧手张开
    #             self.right_hand.move_hand(self.hand_config["left"]["default_standby"])
    #         else: # 否则，灵巧手闭合
    #             self.right_hand.move_hand(self.hand_config["left"]["default_grasp"])
    #         self.last_left_gripper_pct = pct
    
    def publish_actual_angles(event):
        """
        定时调用 get_angle_act 服务，把结果发布到 /hand/angle_actual。
        """
        try:
            rospy.wait_for_service('/inspire_hand_modbus/get_angle_act')
            proxy = rospy.ServiceProxy('/inspire_hand_modbus/get_angle_act', get_angle_act)
            req = get_angle_actRequest()
            req.status = CMD_TYPE
            req.id  = HAND_ID
            resp = proxy(req)
            arr = Int32MultiArray(data=list(resp.curangle))
            pub.publish(arr)
            rospy.loginfo(f"[PUBLISH] actual angles={resp.curangle}")
        except rospy.ServiceException as e:
            rospy.logerr(f"get_angle_act 服务调用失败: {e}")
    
    
    def _on_right_gripper_cmd(self, msg):
        """
        监听 VR Teleop 右夹爪目标位置,根据按下T键来控制灵巧手开合
        """
        pct = msg.position[0]
        if pct != self.last_right_gripper_pct:  # 检查是否发生变化
            rospy.loginfo(f"右夹爪状态变化：{pct}%")
            # 根据百分比（按键触发）决定是开合
            if pct > 50:    # 如果大于50，灵巧手张开
                self.right_hand.move_hand(self.hand_config["right"]["default_standby"])
            else:   # 否则，灵巧手闭合
                self.right_hand.move_hand(self.hand_config["right"]["default_grasp"])
            self.last_right_gripper_pct = pct
    
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
    robot = Galaxea_R1_Interface()
    # hand_pos = robot.right_hand.get_hand_position()
    # print(hand_pos)
    # states = robot.get_states()
    # print("robot_states: ", states)
    # robot.right_hand.move_hand_step(robot.hand_config['right']['default_standby'])
    # image = robot.get_head_image(visulization=True)
    # curr_arm_pos = robot.get_arm_position()
    # print("current_arm_pos: ", curr_arm_pos)
    # target = [0,0,0,0,0,0] #之前测试的结果
    # robot.move_step(target)
    # time.sleep(5)
    # robot.move_to_zero_postion()
    
    # 动一个定时器，每 0.5 秒发布一次实际角度
    rospy.Timer(rospy.Duration(0.5), publish_actual_angles)

    rospy.loginfo(
                " 发布  /hand/angle_actual ← 定时调用 get_angle_act 服务")
    rospy.spin()