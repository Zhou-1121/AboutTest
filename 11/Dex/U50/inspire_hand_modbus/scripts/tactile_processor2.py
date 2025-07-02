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

TOTAL_POINTS = sum(r * c for _, r, c in REGIONS)
BUFFER = []

# 期望的“平均值阈值”
AVG_THRESHOLD = 200
MAX_THRESHOLD = 1000

def process_cycle(cycle):
    """
    把一个完整周期（长度 TOTAL_POINTS)分区、reshape、打印;
    然后对每个分区只算一个平均值，并输出；最后再输出超过阈值的分区。
    """
    mats = {}
    idx = 0
    
    print("\n======== ALL Datas ========")
    rospy.loginfo(f"[tactile_processor] Received raw cycle data ({len(cycle)} pts):")
    for name, rows, cols in REGIONS:
        cnt = rows * cols
        mat = np.array(cycle[idx:idx+cnt], dtype=np.int32).reshape(rows, cols)
        mats[name] = mat
        idx += cnt

    # —— 1. 打印完整矩阵 ——  
    print("\n======== 一周期完整触觉矩阵 ========")
    for name, mat in mats.items():
        print(f"{name} ({mat.shape[0]}x{mat.shape[1]}):\n{mat}\n")

    # —— 2. 逐区计算平均值并输出 ——  
    avgs = {}
    maxs = {}
    for name, mat in mats.items():
        avg = float(np.mean(mat))
        mx  = float(np.max(mat))
        avgs[name] = avg
        maxs[name] = mx
        rospy.loginfo(f"[tactile_processor] {name} 平均值: {avg:.1f}, 最大值: {mx:.1f}")

    # # —— 3. 再筛出超过阈值的区 ——  
    # for name, avg in avgs.items():
    #     if avg > AVG_THRESHOLD:
    #         rospy.loginfo(f"[tactile_processor] 【超阈值-平均】{name} 平均值 {avg:.1f} > {AVG_THRESHOLD}")
    # —— 4. 再筛出超过阈值的区 ——  
    for name, mx in maxs.items():
        if mx > MAX_THRESHOLD:
            rospy.loginfo(f"[tactile_processor] 【超阈值-最大】{name} 最大值 {mx:.1f} > {MAX_THRESHOLD}")

def cb_touch(msg: Int32MultiArray):
    global BUFFER
    BUFFER.extend(msg.data)
    # 只要缓存里够一个整周期，就立刻处理
    while len(BUFFER) >= TOTAL_POINTS:
        cycle = BUFFER[:TOTAL_POINTS]
        BUFFER = BUFFER[TOTAL_POINTS:]
        process_cycle(cycle)

def main():
    rospy.init_node('tactile_processor', anonymous=True)
    rospy.loginfo("[tactile_processor] 节点已启动，订阅 /touch_data …")
    rospy.Subscriber('/touch_data', Int32MultiArray, cb_touch, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    main()