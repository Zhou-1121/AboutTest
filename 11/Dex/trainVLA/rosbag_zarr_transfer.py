import os, argparse, rosbag, time
from bisect import bisect_left
import zarr
import numpy as np
import cv2
import json
from collections import defaultdict
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
from datetime import datetime

ARM_TOPIC   = '/hdas/feedback_arm_right'
HAND_TOPIC  = '/hdas/feedback_gripper_right'
HEAD_TOPIC  = '/hdas/camera_head/left_raw/image_raw_color/compressed'
WRIST_TOPIC = '/hdas/camera_wrist_right/color/image_raw/compressed'

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

def extract_messages(bagfile):
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
            print(f"hand_joint: {msg.position}")
        elif topic == HEAD_TOPIC:
            head_buf.append((rel_t, msg))
        elif topic == WRIST_TOPIC:
            wrist_buf.append((rel_t, msg))

    bag.close()
    actions = pair_actions(arm_buf, hand_buf)
    bridge = CvBridge()
    prev_t = actions[0][0]
    
    head_msgs = []
    wrist_msgs = []
    action_msgs = []
    for idx, (ts, action) in enumerate(actions):
        # —— 按原始节奏 sleep —— 
        dt = (ts - prev_t) 
        if dt>0:
            time.sleep(dt)
        prev_t = ts

        # 找到对应的两张图
        head_msg, head_ts   = find_nearest(head_buf, ts)
        wrist_msg, wrist_ts = find_nearest(wrist_buf, ts)
        head_msgs.append(head_msg)
        wrist_msgs.append(wrist_msg)
        action_msgs.append(action)
        print(f"[{idx:04d}] ts={ts:.3f}s "
            f"| head@{head_ts:.3f}s"
            f"| wrist@{wrist_ts:.3f}s"
            f"| action@{ts:.3f}s -> action = {action}")
    return action_msgs, head_msgs, wrist_msgs


def unscale(x, lower=[-3.1, -2.268, -3.1, -2.355, -3.1, -2.233, -6.28, 0, 0, 0, 0, 0, 0], upper=[3.1, 2.268, 3.1, 2.355, 3.1, 2.233, 6.28, 1000, 1000, 1000, 1000, 1000, 1000]):
    """
    Convert a list of scaled values to [-1, 1].

    Parameters:
    x (list of float): List of scaled values.
    lower (list of float): List of lower bounds for scaling.
    upper (list of float): List of upper bounds for scaling.

    Returns:
    list of float: List of unscaled values.
    """
    return [(xi - lower_i) * 2 / (upper_i - lower_i) - 1 for xi, lower_i, upper_i in zip(x, lower, upper)]

def _crop_bottom_center(img, crop_w=640, crop_h=480):
        """
        从 img 底部居中裁剪 crop_w x crop_h
        """
        h, w = img.shape[:2]
        if crop_w > w or crop_h > h:
            raise ValueError(f"裁剪区域({crop_w}x{crop_h})大于原图({w}x{h})")
        start_x = (w - crop_w) // 2
        start_y = h - crop_h
        # start_y = (h - crop_h) // 2   #中心居中
        return img[start_y:start_y+crop_h, start_x:start_x+crop_w]

# 函数：从 rosbag 中读取数据并保存到 Zarr 格式
def rosbag_to_zarr(bag_files, json_files, zarr_output_dir):
    # 创建 Zarr 数据集
    zarr_store = zarr.open(zarr_output_dir, mode='w')
    
    bridge = CvBridge()
    
    # 创建数据目录
    data_group = zarr_store.create_group('data')
    
    # 初始化数据存储结构
    action_data = []
    rgbm_data = []
    right_cam_img_data = []
    right_state_data = []
    
    # 读取 episode_ends
    episode_ends = []
    end_idx = 0
    # 读取每个 bag 和 json 文件
    for bag_file, json_file in zip(bag_files, json_files):
        action_msgs, head_msgs, wrist_msgs = extract_messages(bag_file)
        for action, head_msg, wrist_msg in zip(action_msgs, head_msgs, wrist_msgs):        
            end_idx += 1
            right_state_data.append(unscale(np.array(action, dtype=np.float32)))
            rgbm_data.append(np.array(_crop_bottom_center(bridge.compressed_imgmsg_to_cv2(head_msg)), dtype=np.uint8))
            right_cam_img_data.append(np.array(bridge.compressed_imgmsg_to_cv2(wrist_msg), dtype=np.uint8))
        episode_ends.append(end_idx)    
    # 将数据存储到 Zarr 数据集
    action_data = np.concatenate((right_state_data[1:], [right_state_data[-1]])) 
    rgbm_data = np.array(rgbm_data, dtype=np.uint8)
    right_cam_img_data = np.array(right_cam_img_data, dtype=np.uint8)
    right_state_data = np.array(right_state_data, dtype=np.float32)
    
    data_group.create_dataset('action', data=action_data)
    data_group.create_dataset('rgbm', data=rgbm_data)
    data_group.create_dataset('right_cam_img', data=right_cam_img_data)
    data_group.create_dataset('right_state', data=right_state_data)
    
    # 创建 meta 数据集
    meta_group = zarr_store.create_group('meta')
    meta_group.create_dataset('episode_ends', data=np.array(episode_ends, dtype=np.int64))
    
    print(f"转换完成，Zarr 文件保存到 {zarr_output_dir}")

# 初始化列表
bag_files = []
json_files = []
directory = '/data/GalaxeaDataset/20250609/20250609_single_tennis/'
# 遍历目录中的文件
for filename in os.listdir(directory):
    # 获取文件的绝对路径
    file_path = os.path.join(directory, filename)
    
    # 确保是文件而不是目录
    if os.path.isfile(file_path):
        # 检查文件扩展名并添加到对应列表
        if filename.endswith('.bag'):
            bag_files.append(file_path)
        elif filename.endswith('.json'):
            json_files.append(file_path)
            
timestamp = datetime.now().strftime("%Y%m%d-%H%M")
zarr_output_dir = f'/data/dingzher/DexGrasp_Demo/SRCB-DexVLA/zarr_data_transfer/output_data_{timestamp}.zarr'

# 执行转换
rosbag_to_zarr(bag_files, json_files, zarr_output_dir)
