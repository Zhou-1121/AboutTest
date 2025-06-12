#!/usr/bin/env python3
import os, argparse, rosbag, time, csv
from bisect import bisect_left
from cv_bridge import CvBridge
import cv2

# ———— 配置区 ————
ARM_TOPIC   = '/hdas/feedback_arm_right'
HAND_TOPIC  = '/hdas/feedback_gripper_right'
HEAD_TOPIC  = '/hdas/camera_head/left_raw/image_raw_color/compressed'
WRIST_TOPIC = '/hdas/camera_wrist_right/color/image_raw/compressed'
# 输出图像的根目录
OUTPUT_DIR = '/home/samsung/qingwen/Hz_duiqi/data_Processing/fixed10hz'

# 采样频率（Hz）
FIXED_HZ = 10.0

# ———— 结束 ————

def extract_messages(bagfile):
    """一次遍历，把四路数据按 (rel_t, data) 缓存到列表里。"""
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
    # 确保升序  arm_buf每一项都是一个二元组（rel_t，data）
    return (sorted(arm_buf,   key=lambda x: x[0]),
            sorted(hand_buf,  key=lambda x: x[0]),
            sorted(head_buf,  key=lambda x: x[0]),
            sorted(wrist_buf, key=lambda x: x[0]),
            start_ts,
            bag.get_end_time() - start_ts)
    # return arm_buf, hand_buf, head_buf, wrist_buf

def find_nearest(buf, ts):
    """
    在已排好序的 buf=[(t,msg),…] 里找最接近 ts 的条目，
    返回 (t,msg)。
    """
    times = [b[0] for b in buf]
    idx = bisect_left(times, ts)    
    # 比较 idx 和 idx-1 哪个更近
    candidates = []
    if idx < len(buf):  candidates.append(buf[idx])
    if idx-1 >= 0:      candidates.append(buf[idx-1])
    return min(candidates, key=lambda x: abs(x[0] - ts)) # 选距离最短的

def align_fixed_hz(bagfile, hz):
    arm_buf, hand_buf, head_buf, wrist_buf, start_ts, duration = extract_messages(bagfile)
    dt = 1.0 / hz
    # 生成相对时间序列
    sample_ts = [i*dt for i in range(int(duration/dt) + 1)]

    # 准备输出目录
    head_dir  = os.path.join(OUTPUT_DIR, 'head')
    wrist_dir = os.path.join(OUTPUT_DIR, 'wrist')
    os.makedirs(head_dir,  exist_ok=True)
    os.makedirs(wrist_dir, exist_ok=True)


    bridge = CvBridge()

    csv_path = os.path.join(OUTPUT_DIR, 'action.csv')
    csv_file = open(csv_path, 'w' , newline='')
    writer   = csv.writer(csv_file)
    writer.writerow(
        ['frame','time_s'] +
        [f'arm{i}' for i in range(7)] +
        [f'hand{i}' for i in range(6)]
    )

    print(f"固定 {hz} Hz 采样 → 总时长 {duration:.3f}秒;共 {len(sample_ts)} 帧，输出到 {OUTPUT_DIR}")

    for idx, rel_t in enumerate(sample_ts):
        # 找到 4 路最近邻
        t_arm,   arm_data   = find_nearest(arm_buf,   rel_t)
        t_hand,  hand_data  = find_nearest(hand_buf,  rel_t)
        t_head,  head_msg   = find_nearest(head_buf,  rel_t)
        t_wrist, wrist_msg  = find_nearest(wrist_buf, rel_t)

        # 保存图像
        head_img  = bridge.compressed_imgmsg_to_cv2(head_msg)
        wrist_img = bridge.compressed_imgmsg_to_cv2(wrist_msg)
        head_fp   = os.path.join(head_dir,  f"frame{idx:04d}_{rel_t:.3f}.png")
        wrist_fp  = os.path.join(wrist_dir, f"frame{idx:04d}_{rel_t:.3f}.png")
        cv2.imwrite(head_fp,  head_img)
        cv2.imwrite(wrist_fp, wrist_img)

        # 合并动作向量
        action = arm_data + hand_data
        writer.writerow([idx, f"{rel_t:.6f}"] + [f"{v:.6f}" for v in action])

        # 打印对齐结果
        print(f"[{idx:04d}] t={rel_t:.3f}s"
              f" | arm@{t_arm:.3f}s"
              f" | hand@{t_hand:.3f}s"
              f" | head@{t_head:.3f}s → {head_fp}"
              f" | wrist@{t_wrist:.3f}s → {wrist_fp}"
              f" | action={action}")

    csv_file.close()
    print("全部导出完成。")
    print(f"动作action已保存到 {csv_path}")

if __name__ == "__main__":
    p = argparse.ArgumentParser(
        description="对齐四路数据到固定采样频率"
    )
    p.add_argument("bagfile", help="rosbag 文件路径")
    p.add_argument("-o","--out",   default=OUTPUT_DIR,
                    help="输出根目录")
    p.add_argument("-f","--freq",  type=float, default=FIXED_HZ,
                    help="固定采样频率 (Hz)")
    args = p.parse_args()

    OUTPUT_DIR = args.out
    align_fixed_hz(args.bagfile, args.freq)



#可以动态调整采样频率和输出目录，不必每次修改源代码：./script.py -f 10.0 -o /my/output /path/to/my.bag
# python data_duiqi_10hz_V1.0.py -f 10.0 -o /home/samsung/qingwen/Hz_duiqi/data_Processing/fixed10hz /home/samsung/qingwen/Hz_duiqi/None_20250430210453852_RAW.bag
# python data_duiqi_10hz_V1.0.py /home/samsung/qingwen/Hz_duiqi/None_20250430205806026_RAW.bag