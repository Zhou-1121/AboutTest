#!/usr/bin/env python3
import os, argparse, rosbag, time
from bisect import bisect_left
from cv_bridge import CvBridge
import cv2

# ———— 配置区 ————
ARM_TOPIC   = '/hdas/feedback_arm_right'
HAND_TOPIC  = '/hdas/feedback_gripper_right'
HEAD_TOPIC  = '/hdas/camera_head/left_raw/image_raw_color/compressed'
WRIST_TOPIC = '/hdas/camera_wrist_right/color/image_raw/compressed'
# 输出图像的根目录
OUTPUT_DIR = '/home/samsung/qingwen/Hz_duiqi/data_Processing/test1'
# ———— 结束 ————

def extract_messages(bagfile):  #先一遍 遍历bag，把（rel_t,data）缓存到四个缓冲区
    bag = rosbag.Bag(bagfile, 'r')
    start_ts = bag.get_start_time()

    arm_buf     = []    # (ts, [7])
    hand_buf    = []    # (ts, [6])
    head_buf    = []    # (ts, msg)
    wrist_buf   = []    # (ts, msg)

    for topic, msg, t in bag.read_messages():
        rel_t = t.to_sec() - start_ts

        if topic == ARM_TOPIC:
            arm_buf.append((rel_t, list(msg.position)))
        elif topic == HAND_TOPIC:
            hand_buf.append((rel_t, list(msg.position)))
        elif topic == HEAD_TOPIC:
            head_buf.append((rel_t, msg))
        elif topic == WRIST_TOPIC:
            wrist_buf.append((rel_t, msg))

    bag.close()
    return arm_buf, hand_buf, head_buf, wrist_buf

def pair_actions(arm_buf, hand_buf):
    """
    按时间先后，把 arm 和 hand 配对，每次取它们的 max 时间做 ts。
    """
    i, j = 0, 0
    actions = []
    while i < len(arm_buf) and j < len(hand_buf):
        t_a, a = arm_buf[i]
        t_h, h = hand_buf[j]
        # 先配对时间更早的那一边
        if abs(t_a - t_h) < 0.001:
            ts = max(t_a, t_h)
            actions.append((ts, a + h))
            i += 1
            j += 1
        elif t_a < t_h:
            i += 1
        else:
            j += 1
    return actions

def find_nearest(buf, ts):
    """
    buf: 已按 ts 升序的 [(ts, msg), ...]
    返回对 ts 最近的 msg
    """
    times = [b[0] for b in buf]
    idx = bisect_left(times, ts)
    # 比较 idx 和 idx-1 哪个更近
    candidates = []
    if idx < len(buf):
        candidates.append(buf[idx])
    if idx-1 >= 0:
        candidates.append(buf[idx-1])
    # 选距离最短的
    best = min(candidates, key=lambda x: abs(x[0]-ts))
    return best[1], best[0]

def align_and_export(bagfile, rate=1.0):
    arm_buf, hand_buf, head_buf, wrist_buf = extract_messages(bagfile)
    actions = pair_actions(arm_buf, hand_buf)
    print(f"配对得到 {len(actions)} 帧 arm+hand 动作")

    # 为对齐输出创建目录
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    os.makedirs(os.path.join(OUTPUT_DIR,'head'), exist_ok=True)
    os.makedirs(os.path.join(OUTPUT_DIR,'wrist'), exist_ok=True)

    bridge = CvBridge()
    prev_t = actions[0][0]

    for idx, (ts, action) in enumerate(actions):
        # —— 按原始节奏 sleep —— 
        dt = (ts - prev_t) / rate
        if dt>0:
            time.sleep(dt)
        prev_t = ts

        # 找到对应的两张图
        head_msg, head_ts   = find_nearest(head_buf, ts)
        wrist_msg, wrist_ts = find_nearest(wrist_buf, ts)

        # 解压并保存
        head_img  = bridge.compressed_imgmsg_to_cv2(head_msg)
        wrist_img = bridge.compressed_imgmsg_to_cv2(wrist_msg)

        head_fn  = os.path.join(OUTPUT_DIR,'head',  f"frame{idx:04d}_{ts:.3f}.png")
        wrist_fn = os.path.join(OUTPUT_DIR,'wrist', f"frame{idx:04d}_{ts:.3f}.png")

        cv2.imwrite(head_fn,  head_img)
        cv2.imwrite(wrist_fn, wrist_img)

         # 打印对齐信息，便于核对
        # print(f"[{idx:04d}] ts={ts:.3f}s | arm+hand(len={len(action)}) "
        #     f"| head@{head_ts:.3f}s → {head_fn} "
        #     f"| wrist@{wrist_ts:.3f}s → {wrist_fn}")
        print(f"[{idx:04d}] ts={ts:.3f}s "
            f"| head@{head_ts:.3f}s → {head_fn} "
            f"| wrist@{wrist_ts:.3f}s → {wrist_fn}"
            f"| arm+hand(len={len(action)}) | action = {action}")

    print("全部导出完成。")

if __name__ == "__main__":
    p = argparse.ArgumentParser(
        description="Align arm+hand actions with head & wrist camera images"
    )
    p.add_argument("bagfile", help="rosbag file")
    p.add_argument("-r","--rate",type=float,default=1.0,
                    help="playback speed factor")
    args = p.parse_args()
    align_and_export(args.bagfile, rate=args.rate)

# python3 data_duiqi.py -r 1.0 /home/samsung/qingwen/scripts/None_20250430223934664_RAW.bag