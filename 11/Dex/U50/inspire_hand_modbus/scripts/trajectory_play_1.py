#!/usr/bin/env python3
import rosbag
import rospy
import cv2
import time
from cv_bridge import CvBridge
from Galaxea_R1_Interface import Galaxea_R1_Interface

BAG_FILE    = '/home/samsung/qingwen/scripts/None_20250508170625110_RAW.bag'

IMAGE_TOPIC  = '/hdas/camera_head/left_raw/image_raw_color/compressed'
ARM_TOPIC    = '/hdas/feedback_arm_right'
HAND_TOPIC   = '/hdas/feedback_gripper_right'

def get_action():
    """
    打开 BAG_FILE,按时间顺序遍历所有消息。
    每当同时拿到一帧图像、一条右臂控制命令、一条右爪反馈时，
    就把它们打包成一个元素,append 到 actions 列表。
    返回：[(cv_image, [7 关节目标], [6 轴实际]), …]
    """
    bridge = CvBridge()
    bag    = rosbag.Bag(BAG_FILE, 'r')

    last_arm  = None    # 缓存临时的右臂命令
    last_hand = None    # 缓存临时的右爪反馈
    actions   = []      # 最终返回的动作列表

    for topic, msg, t in bag.read_messages(): #从rosbag读取动作帧
        if topic == IMAGE_TOPIC:
            # 解压压缩图像，得到 OpenCV 的 BGR numpy 数组
            img = bridge.compressed_imgmsg_to_cv2(msg)
        elif topic == ARM_TOPIC:
            # motor_control.msg 里的目标角度列表，假设在 p_des 字段
            last_arm = list(msg.p_des)
        elif topic == HAND_TOPIC:
            # JointState.position 就是 6 轴角度
            last_hand = list(msg.position)
        # 如果 arm 和 hand 都已经读到，就组成一次完整“动作”
        if last_arm is not None and last_hand is not None:
            actions.append((img, last_arm, last_hand))
            # 清空，等待下一次 new arm+hand
            last_arm  = None
            last_hand = None
    bag.close()
    return actions

if __name__ == "__main__":
    # 初始化 ROS & 机器人接口
    rospy.init_node('trajectory_play', anonymous=True)
    robot = Galaxea_R1_Interface(init=False)
    # 让 OpenCV 弹个窗口，用来显示头部相机
    cv2.namedWindow('Head Camera', cv2.WINDOW_NORMAL)
    # 1) 先把所有动作一次性读出来
    actions = get_action()
    print(f"■ 共读取到 {len(actions)} 帧图+臂+爪 动作")
    # 2) 遍历执行
    for idx, (img, arm_cmd, hand_cmd) in enumerate(actions):
        # 显示图像
        cv2.imshow('Head Camera', img)
        # 按q，中途退出
        if cv2.waitKey(1) == ord('q'):
            break
        # 合并成7+6数值列表，给move_step执行
        action = arm_cmd + hand_cmd
        robot.move_step(action)
        # 机械臂反应时间（可根据需要调大/调小）
        time.sleep(0.1)
    cv2.destroyAllWindows()

#rosbag play /data/yixiang/SRCB-DexVLA/dataset/GalaxeaDataset/20250508/None_20250508170625110_RAW.bag -r 1.0