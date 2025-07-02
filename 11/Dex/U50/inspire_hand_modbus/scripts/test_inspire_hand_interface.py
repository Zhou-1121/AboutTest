#!/usr/bin/env python3
import rospy
from inspire_hand_interface import Inspire_Hand_Interface # 接口类

def main():
    # 在这里初始化 ROS 节点
    rospy.init_node('test_inspire_hand_interface_node')

    # 创建接口实例
    robot = Inspire_Hand_Interface()

    # 定义三种预设动作
    motion1 = [1000, 1000, 1000, 1000, 1000, 1000] # 全展开
    motion2 = [755, 805, 820, 860, 1000, 410]  # 自然状态
    motion3 = [525, 510, 495, 450, 500, 0]  # 抓取状态
    #motion3 = [450, 250, 250, 250, 500, 0]  # 抓取状态

    # 两套速度参数
    speed_fast = [600]*6
    speed_slow = [200]*6

    # 1) 起始：读取当前角度
    rospy.loginfo("获取当前灵巧手角度…")
    curr = robot.get_hand_position()
    rospy.loginfo(f"当前角度: {curr}")

    # 2) 动作1：全展开
    rospy.loginfo("灵巧手全展开状态（快速）")
    robot.move_step(motion1, speed=speed_fast)
    rospy.sleep(0.5)

    # 3) 动作2：自然状态
    rospy.loginfo("灵巧手自然状态（慢速）")
    robot.move_step(motion2, speed=speed_slow)
    rospy.sleep(1.0)

    # 4) 进入输入循环，根据用户选择抓取 / 放下
    while not rospy.is_shutdown():
        choice = input("请输入操作指令 [抓取:1 ; 放下:0 ; 退出:q] >>> ").strip().lower()
        if choice == '1':
            rospy.loginfo("执行抓取状态")
            robot.move_step(motion3,speed=speed_slow)
        elif choice == '0':
            rospy.loginfo("返回动作2:自然状态")
            robot.move_step(motion2,speed=speed_slow)
        elif choice == 'q':
            rospy.loginfo("退出测试")
            break
        else:
            print("无效输入，请输入 1、0 或 q。")
            continue
        # 给机械手一些时间去执行，避免命令过快被覆盖
        rospy.sleep(1.0)

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass