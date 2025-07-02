#!/usr/bin/env python3
import rosbag
import time
import argparse

# —— 配置区 ——#
ARM_TOPIC  = '/hdas/feedback_arm_right'     # JointState, 7 维
HAND_TOPIC = '/hdas/feedback_gripper_right' # JointState, 6 维
# —— 结束配置 ——#

def extract_actions(bagfile):

    bag = rosbag.Bag(bagfile, 'r')
    start_ts = bag.get_start_time()
    last_arm = last_hand = None
    t_arm = t_hand = 0.0
    actions = []

    for topic, msg, t in bag.read_messages():
        rel_t = t.to_sec() - start_ts

        if topic == ARM_TOPIC:
            last_arm = list(msg.position)
            t_arm = rel_t
        elif topic == HAND_TOPIC:
            last_hand = list(msg.position)
            t_hand = rel_t

        # 一旦 arm+hand 都就绪，记录一次
        if last_arm is not None and last_hand is not None:
            ts = max(t_arm, t_hand)
            actions.append((ts, last_arm + last_hand))
            last_arm = last_hand = None

    bag.close()
    return actions

def main():
    p = argparse.ArgumentParser(
        description="Offline 验证 rosbag 中 7+6 数据，按录制节奏打印")
    p.add_argument("bagfile", help="rosbag 文件路径")
    p.add_argument("-r","--rate", type=float, default=1.0,
                    help="播放速率 (1.0=原速；2.0=2×；0.5=慢放)")
    args = p.parse_args()

    actions = extract_actions(args.bagfile)
    print(f"共提取到 {len(actions)} 帧 arm+hand 数据")
    if not actions:
        return

    prev_t = actions[0][0]
    for ts, action in actions:
        # 按原录制节奏 sleep
        dt = (ts - prev_t) / args.rate
        if dt > 0:
            time.sleep(dt)
        prev_t = ts

        # 打印时间戳与 13 维数据
        print(f"[{ts:.3f}s]  {action}")

if __name__ == "__main__":
    main()

# ./trajectory_play_lixian.py -r 1.0 /home/samsung/qingwen/scripts/None_20250430223934664_RAW.bag
