#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from std_msgs.msg import Int32MultiArray

# —— 1. 各触觉区域尺寸 ——  
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
TOTAL_POINTS = sum(rows * cols for _, rows, cols in REGIONS)

buffer = []

def process_cycle(cycle):
    """把单周期 flat list 拆分、reshape、打印并阈值检测。"""
    mats = {}
    idx = 0
    for name, rows, cols in REGIONS:
        cnt = rows * cols
        mat = np.array(cycle[idx:idx+cnt], dtype=np.int32).reshape(rows, cols)
        mats[name] = mat
        idx += cnt

    print("\n======== 一周期完整触觉矩阵 ========")
    for name, mat in mats.items():
        print(f"{name} ({mat.shape[0]}x{mat.shape[1]}):\n{mat}\n")

    TH = 50
    for name, mat in mats.items():
        pts = np.argwhere(mat > TH)
        if pts.size:
            rospy.loginfo(f"[tactile_processor] {name} 超阈值点 (行,列):{pts.tolist()}")

def cb_touch(msg: Int32MultiArray):
    global buffer
    data = msg.data
    buffer.extend(data)

    # 只要缓存里有足够的点，就拿出一整周期处理
    while len(buffer) >= TOTAL_POINTS:
        cycle = buffer[:TOTAL_POINTS]
        buffer = buffer[TOTAL_POINTS:]
        process_cycle(cycle)

def main():
    rospy.init_node('tactile_processor', anonymous=True)
    rospy.loginfo("[tactile_processor] 节点已启动，订阅 /touch_data …")
    rospy.Subscriber('/touch_data', Int32MultiArray, cb_touch, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    main()
    
    
    
    