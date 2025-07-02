#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32MultiArray
from inspire_hand_modbus.srv import (
    set_angle, set_angleRequest,
    get_angle_act, get_angle_actRequest,
    set_speed, set_speedRequest
)

class Inspire_Hand_Interface():
    def __init__(self):
        #在这部分检查是否已经完成ROS节点初始化、
        if not rospy.core.is_initialized():
            rospy.init_node('inspire_hand_interface') # 初始化ROS节点
            
        self.rate = rospy.Rate(10)
        # 发布实际角度的Topic
        self.pub_hand_position = rospy.Publisher('/hand/set_cmd', Int32MultiArray, queue_size = 1)

        # 订阅实际角度的Topic
        self.sub_hand_position = rospy.Subscriber('/hand/angle_actual', Int32MultiArray, self.hand_position_callback)

        # 用来存储当前的手部角度
        self.current_hand_position = None

    def hand_position_callback(self, msg):
        """
        回调函数，接收到 /hand/angle_actual 发布的消息时触发
        这里更新当前的手部角度
        """
        self.current_hand_position = list(msg.data)
        #rospy.loginfo(f"当前灵巧手角度: {self.current_hand_position}")

    def get_hand_position(self):
        """
        获取当前的灵巧手角度：
        一次性阻塞，直到收到 /hand/angle_actual 的一条消息后返回。
        """
        try:
            msg = rospy.wait_for_message('/hand/angle_actual', Int32MultiArray)
            return list(msg.data)
        except rospy.ROSException as e:
            rospy.logerr(f"等待 /hand/angle_actual 消息超时或出错: {e}")
            return None
        
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

    def move_step(self, target, speed=None):
        """
        控制灵巧手移动：
        - 可选:先设置速度(speed_list)
        - 再设置角度(target)
        """
        # 1) 如果传了 speed，就先设置速度
        if speed is not None:
            ok = self.set_speed(speed)
            if not ok:
                rospy.logwarn("速度设置失败，跳过速度调用")
            rospy.sleep(0.1)    # 给速度设置一点时间

        # 2) 发布角度命令到 Topic（如果还需要这一层桥接）
        rospy.loginfo(f"发布目标角度：{target}")
        msg = Int32MultiArray(data=target)
        self.pub_hand_position.publish(msg)

        # 3) 同步调用 set_angle 服务
        try:
            rospy.wait_for_service('/inspire_hand_modbus/set_angle')
            proxy = rospy.ServiceProxy('/inspire_hand_modbus/set_angle', set_angle)
            req = set_angleRequest()
            req.status = ""     # 如果你的 .srv 里有 status 字段，请填
            req.id   = 1     # 灵巧手 ID
            req.angle0, req.angle1, req.angle2, req.angle3, req.angle4, req.angle5 = target
            resp = proxy(req)
            rospy.loginfo(f"设置角度：{target} → accepted={resp.angle_accepted}")
            return resp.angle_accepted
        except rospy.ServiceException as e:
            rospy.logerr(f"调用 set_angle 服务失败: {e}")
            return False

        
if __name__ == "__main__":
    # 创建Inspire_Hand_Interface类实例
    robot = Inspire_Hand_Interface()

    # 1) 先设置速度（全慢一点，比如 200）
    speeds = [2, 200, 200, 200, 200, 200]
    robot.set_speed(speeds)
    rospy.sleep(0.5)

    # 2) 然后设置一个角度
    target =[1000, 1000, 1000, 1000, 1000, 1000]
    robot.move_step(target)

    # 3) 获取并打印实际角度
    actual = robot.get_hand_position()
    rospy.loginfo(f"实际角度: {actual}")