#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#数据可视化，平滑处理, 基准校准
import rospy
import numpy as np
import threading
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from std_msgs.msg import Int32MultiArray
from collections import deque

# —— 各区域定义 ——  
REGIONS = [
    ('小拇指指端', 3, 3),
    ('小拇指指尖',12, 8),
    ('小拇指指腹',10, 8),
    ('无名指指端', 3, 3),
    ('无名指指尖',12, 8),
    ('无名指指腹',10, 8),
    ('中指指端', 3, 3),
    ('中指指尖',12, 8),
    ('中指指腹',10, 8),
    ('食指指端', 3, 3),
    ('食指指尖',12, 8),
    ('食指指腹',10, 8),
    ('大拇指指端', 3, 3),
    ('大拇指指尖',12, 8),
    ('大拇指指中', 3, 3),
    ('大拇指指腹',12, 8),
    ('掌心',    8,14),
]
TOTAL_POINTS = sum(r * c for _, r, c in REGIONS)

# —— 滑动平均参数 ——  
SMOOTH_FRAMES = 5
history = {
    name: deque(maxlen=SMOOTH_FRAMES)
    for name, _, _ in REGIONS
}

# —— 启动时基线采集 ——  
BASELINE_FRAMES = 10
baseline_history = []   # 存前 BASELINE_FRAMES 帧的原始 mats
baseline_mats = None    # 计算出的基线矩阵
baseline_ready = False  # 标记基线是否采集完毕

lock = threading.Lock()
latest_mats = None
new_data = False

def parse_cycle_to_mats(cycle):
    """把一维 cycle 拆成每个子矩阵 dict"""
    mats = {}
    idx = 0
    for name, rows, cols in REGIONS:
        cnt = rows * cols
        mat = np.array(cycle[idx:idx+cnt], dtype=np.int32).reshape(rows, cols)
        mats[name] = mat
        idx += cnt
    return mats

def cb_touch(msg: Int32MultiArray):
    global latest_mats, new_data
    global baseline_history, baseline_mats, baseline_ready

    data = list(msg.data)
    if len(data) != TOTAL_POINTS:
        return  # 丢弃不对齐的数据

    mats = parse_cycle_to_mats(data)

    # —— 1. 基线采集阶段 ——  
    if not baseline_ready:
        baseline_history.append(mats)
        rospy.loginfo(f"[tactile_processor] 采集基线帧 {len(baseline_history)}/{BASELINE_FRAMES}")
        if len(baseline_history) >= BASELINE_FRAMES:
            # 计算每个区域的基线平均矩阵
            baseline_mats = {}
            for name, _, _ in REGIONS:
                stack = np.stack([h[name] for h in baseline_history], axis=0)
                baseline_mats[name] = np.mean(stack, axis=0)
            baseline_ready = True
            rospy.loginfo("[tactile_processor] 基线采集完成，开始实时去基线+可视化")
        return

    # —— 2. 去基线并 clip ——  
    for name in mats:
        mats[name] = mats[name] - baseline_mats[name]
        mats[name][mats[name] < 0] = 0

    # —— 3. 帧内滑动平均 ——  
    for name in mats:
        history[name].append(mats[name])
        # 每个 deque 里保存最近 SMOOTH_FRAMES 帧，取平均
        mats[name] = np.mean(history[name], axis=0)

    # —— 4. 更新全局最新数据 ——  
    with lock:
        latest_mats = mats
        new_data = True

def init_plot():
    fig = plt.figure(figsize=(10,12))
    gs = fig.add_gridspec(5, 5, wspace=0.3, hspace=0.4)
    layout = {
        '小拇指指端': (0, 0, 1, 1),
        '无名指指端': (0, 1, 1, 1),
        '中指指端':   (0, 2, 1, 1),
        '食指指端':   (0, 3, 1, 1),
        '大拇指指端': (0, 4, 1, 1),

        '小拇指指尖': (1, 0, 1, 1),
        '无名指指尖': (1, 1, 1, 1),
        '中指指尖':   (1, 2, 1, 1),
        '食指指尖':   (1, 3, 1, 1),
        '大拇指指尖': (1, 4, 1, 1),

        '小拇指指腹': (2, 0, 1, 1),
        '无名指指腹': (2, 1, 1, 1),
        '中指指腹':   (2, 2, 1, 1),
        '食指指腹':   (2, 3, 1, 1),
        '大拇指指中': (2, 4, 1, 1), # thumb 中段

        '大拇指指腹': (3, 4, 1, 1),

        '掌心':     (4, 0, 1, 5),   # 跨 5 列
    }
    axes_map = {}
    for name, (r, c, rs, cs) in layout.items():
        ax = fig.add_subplot(gs[r:r+rs, c:c+cs])
        ax.set_title(name, fontsize=8)
        ax.axis('off')
        axes_map[name] = ax

    cbar_ax = fig.add_axes([0.15, 0.05, 0.7, 0.02])
    fig.suptitle("实时触觉热力图（手形布局）", fontsize=14)
    return fig, axes_map, cbar_ax

def update_plot(fig, ax_map, mats, cbar_ax):
    for name, mat in mats.items():
        ax = ax_map[name]
        ax.clear()
        im = ax.imshow(mat,
                        vmin=0, vmax=10000,
                        interpolation='nearest',
                        aspect='auto')
        ax.set_title(name, fontsize=8)
        ax.axis('off')
    fig.colorbar(im, cax=cbar_ax, orientation='horizontal')
    fig.canvas.draw_idle()

def main():
    rospy.init_node('tactile_processor', anonymous=True)
    rospy.Subscriber('/touch_data', Int32MultiArray, cb_touch, queue_size=1)
    rospy.loginfo("[tactile_processor] 节点已启动，订阅 /touch_data")

    # ROS spin in 子线程，主线程跑 GUI
    spinner = threading.Thread(target=rospy.spin)
    spinner.daemon = True
    spinner.start()

    fig, axes_map, cbar_ax = init_plot()
    while not rospy.is_shutdown():
        updated = False
        with lock:
            global new_data
            if new_data:
                mats = latest_mats
                new_data = False
                updated = True
        if updated:
            update_plot(fig, axes_map, mats, cbar_ax)
        plt.pause(0.05)

    plt.close(fig)

if __name__ == '__main__':
    main()