#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#数据可视化，平滑处理, 基准校准
import rospy
import numpy as np
import threading
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib import rcParams
# plt.rcParams['font.sans-serif'] = ['WenQuanYi Zen Hei']
# plt.rcParams['axes.unicode_minus'] = True
plt.rcParams['font.family'] = 'sans-serif'
plt.rcParams['font.sans-serif'] = ['WenQuanYi Zen Hei','Noto Sans CJK SC']
plt.rcParams['axes.unicode_minus'] = False

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

# 滑动平均参顺
SMOOTH_FRAMES= 5 #平滑窗口大小
history = { name: deque(maxlen= SMOOTH_FRAMES) for name, _, _ in REGIONS }

# 启动时基线采集
BASELINE_FRAMES = 5     #N帧
baseline_hisotry = []   #存前N帧的原始mats
baseline_mats = None    #计算出基线矩阵
baseline_ready = False  #用于标记基线是否采集完成

# 全局缓存最新一帧数据的矩阵
latest_mats = None
new_data = False
lock = threading.Lock()

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
    data = list(msg.data)
    # 假设每条消息正好完整一个周期
    if len(data) == TOTAL_POINTS:
        mats = parse_cycle_to_mats(data)
        with lock:
            for name, mat in mats.items():
                history[name].append(mat)
            new_data = True
    # else:
    #     rospy.logwarn(f"[tactile_processor] 数据长度 {len(data)} != {TOTAL_POINTS}，已丢弃")

def init_plot():
    fig = plt.figure(figsize=(10,12))
    gs = fig.add_gridspec(5, 5, wspace=0.3, hspace=0.4)

    # 预先写好每个 region 在 gridspec 中的位置：(row, col, rowspan, colspan)
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
    # 参数[left,bottom,width,height]都是0-1的比例
    cbar_ax = fig.add_axes([0.15, 0.05, 0.7, 0.02])

    fig.suptitle("实时触觉热力图（手形布局）", fontsize=14)
    return fig, axes_map, cbar_ax

def update_plot(fig, ax_map, mats, cbar_ax):
    for name, mat in mats.items():
        ax = ax_map[name]
        ax.clear()
        im = ax.imshow(mat,
                        vmin=0, vmax=20000,     # 根据你的量程自行调整
                        interpolation='nearest',
                        aspect='auto')
        ax.set_title(name, fontsize=8)
        ax.axis('off')

    # 如果想要统一放一个 Colorbar，可以：
    fig.colorbar(im, cax=cbar_ax, orientation='horizontal')
    
    fig.canvas.draw_idle()

def main():
    rospy.init_node('tactile_processor', anonymous=True)
    rospy.Subscriber('/touch_data', Int32MultiArray,
                    cb_touch, queue_size=1)
    rospy.loginfo("[tactile_processor] 节点已启动，订阅 /touch_data")

    # ROS spin 放子线程，保证主线程跑 GUI loop
    spinner = threading.Thread(target=rospy.spin)
    spinner.daemon = True
    spinner.start()

    fig, axes_map, cbar_ax = init_plot()
    while not rospy.is_shutdown():
        updated = False
        with lock:
            global new_data
            if new_data:
                #构造一个‘平滑后’的mats
                smooth_mats = {}
                for name in history:
                    # 如果队列未满，就用已有的帧来平均
                    arrs = list(history[name])
                    smooth_mats[name] = np.mean(arrs, axis=0)
                new_data = False
                updated = True
        if updated:
            update_plot(fig, axes_map, smooth_mats, cbar_ax)
        # pause 既刷新图又让 GUI 事件循环跑起来
        plt.pause(0.05)

    plt.close(fig)

if __name__ == '__main__':
    main()