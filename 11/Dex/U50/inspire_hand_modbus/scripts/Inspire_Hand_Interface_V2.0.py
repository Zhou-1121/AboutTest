#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Int32MultiArray
from inspire_hand_modbus.srv import (
    set_angle, set_angleRequest,
    get_angle_act, get_angle_actRequest
)

CMD_TYPE = ""
HAND_ID = 1

#订阅回调：接受设置命令”
#订阅：
def set_angle_cb(msg: Int32MultiArray):
    """
    当在 /hand/set_cmd 话题上收到角度命令(6个整数)时,
    就调用 set_angle 服务一次。
    """
    angles = list(msg.data)
    if len(angles) != 6:
        rospy.logwarn(f"收到无效命令长度: {len(angles)}，跳过")
        return

    try:
        rospy.wait_for_service('/inspire_hand_modbus/set_angle')
        proxy = rospy.ServiceProxy('/inspire_hand_modbus/set_angle', set_angle)
        req = set_angleRequest()
        req.status  = CMD_TYPE
        req.id  = HAND_ID
        req.angle0, req.angle1, req.angle2, req.angle3, req.angle4, req.angle5 = angles
        resp = proxy(req)
        rospy.loginfo(f"[SET_CMD] angles={angles} → accepted={resp.angle_accepted}")
    except rospy.ServiceException as e:
        rospy.logerr(f"set_angle 服务调用失败: {e}")

#
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

if __name__ == "__main__":
    rospy.init_node('hand_pubsub_bridge')

    # 1) 订阅外部的“设置命令”话题
    rospy.Subscriber('/hand/set_cmd', Int32MultiArray, set_angle_cb)

    # 2) 发布实际角度的 Topic
    pub = rospy.Publisher('/hand/angle_actual', Int32MultiArray, queue_size=10)

    # 3) 启动一个定时器，每 0.5 秒发布一次实际角度
    rospy.Timer(rospy.Duration(0.5), publish_actual_angles)

    rospy.loginfo("hand_pubsub_bridge 启动：\n"
                " 订阅 /hand/set_cmd → 调用 set_angle 服务\n"
                " 发布  /hand/angle_actual ← 定时调用 get_angle_act 服务")
    rospy.spin()



