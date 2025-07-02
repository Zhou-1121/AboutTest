#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#引入 ROS Python 客户端库
#导入两对服务类型：
# `set_angle`服务及其请求类型`set_angleRequest`  
# `get_angle_act` 服务及其请求类型 `get_angle_actRequest`  
import rospy
from inspire_hand_modbus.srv import (
    set_angle, set_angleRequest,
    get_angle_act, get_angle_actRequest
)

CMD_TYPE = "" # 灵巧手所需的命令类型,srv中提到的string status
HAND_ID = 1  # 灵巧手 ID（int16）

# "设置角度”函数
def set_hand_angle(angle_list):
    """调用 set_angle 服务一次，打印结果后返回 True/False"""
    rospy.wait_for_service('/inspire_hand_modbus/set_angle')
    try:
        proxy = rospy.ServiceProxy('/inspire_hand_modbus/set_angle', set_angle)
        req = set_angleRequest()
        req.status  = CMD_TYPE
        req.id  = HAND_ID
        req.angle0, req.angle1, req.angle2, req.angle3, req.angle4, req.angle5 = angle_list
        resp = proxy(req)
        rospy.loginfo(f"[SET] angles={angle_list} → accepted={resp.angle_accepted}")
        return resp.angle_accepted
    except rospy.ServiceException as e:
        rospy.logerr(f"set_angle 调用失败: {e}")
        return False


# “读取实际角度”函数
def get_hand_angle(event=None):
    """调用 get_angle_act 服务一次，打印当前角度"""
    rospy.wait_for_service('/inspire_hand_modbus/get_angle_act')
    try:
        proxy = rospy.ServiceProxy('/inspire_hand_modbus/get_angle_act', get_angle_act)
        req = get_angle_actRequest()
        req.status = CMD_TYPE
        req.id  = HAND_ID
        resp = proxy(req)
        angles = tuple(resp.curangle)
        rospy.loginfo(f"[GET] actual angles={angles}")
    except rospy.ServiceException as e:
        rospy.logerr(f"get_angle_act 调用失败: {e}")

def main():
    # 初始化 ROS 节点，名字叫 `hand_cli_mode`,`anonymous=True` 允许多次运行而名字自动加后缀，避免冲突。但不马上启动定时器
    rospy.init_node('hand_cli_mode', anonymous=True)

    # 1) 让用户选择模式
    mode = None
    while mode not in ('r', 's'):
        mode = input("请选择模式：[r] 读取角度 [s] 设置角度 > ").strip().lower()

    if mode == 'r':
        # 读取模式：每 0.5 秒调用一次 get_hand_angle
        rospy.loginfo("进入读取模式，每 0.5 秒获取一次灵巧手角度(Ctrl+C 停止）")
        rospy.Timer(rospy.Duration(0.5), get_hand_angle)
        rospy.spin()
        
    else:
        # 设置模式：提示输入 6 个逗号分隔的整数
        # ====== 这里改成循环 ======
        while True:
            text = input("请输入 6 个逗号分隔的角度值 (0-1000)，如 1000,10,50,20,60,40  > ")
            try:
                parts = text.split(',')
                if len(parts) != 6:
                    raise ValueError("必须输入 6 个数值")
                angles = [int(x) for x in parts]
                if any(a < 0 or a > 1000 for a in angles):
                    raise ValueError("数值必须在 0-1000 范围内")
            except ValueError as e:
                print(f"输入有误：{e}，请重新输入。")
                continue

            ok = set_hand_angle(angles)
            print("设置成功" if ok else "设置失败")

            # 询问是否继续
            choice = input("是否继续设置新的角度？[y/n]  > ").strip().lower()
            if choice not in ('y', 'yes'):
                print("退出设置模式。")
                break
        # ====== 循环结束，脚本也会结束 ======


if __name__ == "__main__":
    main()