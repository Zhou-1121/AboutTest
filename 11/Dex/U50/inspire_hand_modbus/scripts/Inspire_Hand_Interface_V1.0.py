#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from inspire_hand_modbus.srv import (
    set_angle, set_angleRequest,
    get_angle_act, get_angle_actRequest
)
import random

# —————— 需要根据自己的设备参数来填写 ——————
CMD_TYPE = ""  # srv 文件中的 status 字段：命令类型字符串
HAND_ID = 1  # srv 文件中的 id 字段：灵巧手 ID（int16）
# ————————————————————————————————————————————————

def set_hand_angle(angle_list):
    rospy.wait_for_service('/inspire_hand_modbus/set_angle')
    try:
        proxy = rospy.ServiceProxy('/inspire_hand_modbus/set_angle', set_angle)
        req = set_angleRequest()
        req.status = CMD_TYPE
        req.id  = HAND_ID
        # 拆包赋值
        req.angle0,req.angle1,req.angle2,req.angle3,req.angle4,req.angle5 = angle_list
        resp = proxy(req)
        rospy.loginfo(f"[SET] angles={angle_list} → accepted={resp.angle_accepted}")
        return resp.angle_accepted
    except rospy.ServiceException as e:
        rospy.logerr(f"set_angle 调用失败: {e}")
        return False
    
def get_hand_angle():
    """
    调用 /inspire_hand_modbus/get_angle_act 服务，
    返回: 长度为6的 tuple,实际角度；失败则返回 None
    """
    rospy.wait_for_service('/inspire_hand_modbus/get_angle_act')
    try:
        proxy = rospy.ServiceProxy('/inspire_hand_modbus/get_angle_act', get_angle_act)
        req = get_angle_actRequest()
        req.status = CMD_TYPE
        req.id  = HAND_ID
        resp = proxy(req)
        angles = tuple(resp.curangle)
        rospy.loginfo(f"[GET] actual angles={angles}")
        return angles
    except rospy.ServiceException as e:
        rospy.logerr(f"get_angle_act 调用失败: {e}")
        return None

def timer_cb(event):
    target = [ random.randint(0,1000) for _ in range(6) ]
    if set_hand_angle(target):
        get_hand_angle()

if __name__=="__main__":
    rospy.init_node('hand_scheduler')
    rospy.Timer(rospy.Duration(5.0), timer_cb)
    rospy.loginfo("手调度节点启动,每5秒设置并读取一次角度")
    rospy.spin()