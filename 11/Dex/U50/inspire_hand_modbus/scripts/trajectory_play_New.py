#!/usr/bin/env python3
import rosbag
import rospy
import time
import argparse  #用于命令项选项与参数解析的模块
from Galaxea_R1_Interface import Galaxea_R1_Interface

# BAG_FILE    = '/home/samsung/qingwen/scripts/None_20250430223934664_RAW.bag'
ARM_TOPIC    = '/hdas/feedback_arm_right'       # 7 轴 JointState
HAND_TOPIC   = '/hdas/feedback_gripper_right'   # 6 轴 JointState

def parse_args():
    p = argparse.ArgumentParser(
        description="Realtime replay of 7+6 feedback on real robot"
    )
    p.add_argument("bagfile",
                    help="Path to your .bag file")
    p.add_argument("-r","--rate", type=float, default=1.0,
                    help="Playback speed factor; 1.0=原速")
    return p.parse_args()

def get_action(bagfile):
    """
    从 bag 中读取 ARM_TOPIC + HAND_TOPIC,
    每当 arm+hand 同时出现时，保存：(ts, action_list)
    其中 ts 是该帧相对 bag 起点的秒数，
    action_list 是 [7 arm angles + 6 hand angles]。
    """
    bag    = rosbag.Bag(bagfile, 'r')
    start_ts = bag.get_start_time()
    last_arm  = None    # 缓存临时的右臂命令
    last_hand = None    # 缓存临时的右爪反馈
    t_arm = t_hand = 0.0
    actions   = []      # 最终返回的动作列表

    for topic, msg, t in bag.read_messages(): #从rosbag读取动作帧
        rel_t = t.to_sec() - start_ts

        if  topic == ARM_TOPIC:
            # 读取 JointState.position里面的7轴关节反馈
            last_arm = list(msg.position)
            t_arm = rel_t
        elif topic == HAND_TOPIC:
            # JointState.position 就是 6 轴角度
            last_hand = list(msg.position)
            t_hand = rel_t
        # 如果 arm 和 hand 都已经读到，就组成一次完整“动作”,，取它们的max时间作为这一帧的时间
        if last_arm is not None and last_hand is not None:
            ts = max(t_arm, t_hand)
            actions.append((ts, last_arm + last_hand))
            # 清空，等待下一次 new arm+hand
            last_arm  = None
            last_hand = None
    bag.close()
    return actions

if __name__ == "__main__":
    args = parse_args()
    # 1) 先提取全部动作帧
    actions = get_action(args.bagfile)
    print(f"■ 共读取到 {len(actions)} 帧臂+爪动作")
    if not actions:
        print("没有找到任何动作帧，退出。")
        exit(0)
    # 2) 初始化 ROS & 机器人接口
    rospy.init_node('trajectory_play', anonymous=True)
    robot = Galaxea_R1_Interface(init=False)
    # 3) 按时间戳 / 速率 回放
    prev_t = actions[0][0]
    for t, action in actions:
        # 计算两帧间隔，除以 rate 实现加速/减速
        dt = (t - prev_t) / args.rate
        if dt > 0:
            time.sleep(dt)
        prev_t = t    
        print(action)
        robot.move_step(action)
    print("▶ 回放完毕。")

# #rosbag play /data/yixiang/SRCB-DexVLA/dataset/GalaxeaDataset/20250508/None_20250508170625110_RAW.bag -r 1.0
# ./trajectory_play_real.py -r 1.0 /data/yixiang/SRCB-DexVLA/dataset/GalaxeaDataset/20250508/None_20250430223934664_RAW.bag


