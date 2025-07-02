#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#未实现可视化
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

print(f"======== TOTAL_POINTS:{TOTAL_POINTS} ========")
# 阈值参数
AVG_THRESHOLD = 1000
MAX_THRESHOLD = 2000

def process_cycle(cycle):
    # —— 0. 原始数据（和 rostopic echo 完全一致） —— 
    rospy.loginfo(f"[tactile_processor] Received raw cycle data ({len(cycle)} pts): {cycle}")

    # —— 1. 拆分 + reshape —— 
    mats = {}
    idx = 0
    for name, rows, cols in REGIONS:
        cnt = rows * cols
        mat = np.array(cycle[idx:idx+cnt], dtype=np.int32).reshape(rows, cols)
        mats[name] = mat
        idx += cnt

    # —— 2. 打印所有矩阵 —— 
    print("\n======== 一周期完整触觉矩阵 ========")
    for name, mat in mats.items():
        print(f"{name} ({mat.shape[0]}x{mat.shape[1]}):\n{mat}\n")

    # —— 3. 计算平均值 & 最大值 —— 
    avgs = {}
    maxs = {}
    for name, mat in mats.items():
        avg = float(np.mean(mat))
        mx  = float(np.max(mat))
        avgs[name] = avg
        maxs[name] = mx
        rospy.loginfo(f"[tactile_processor] {name} 平均值: {avg:.1f}, 最大值: {mx:.1f}")

    # —— 4. 输出超阈值项 —— 
    # for name, avg in avgs.items():
        # if avg > AVG_THRESHOLD:
        #     rospy.loginfo(f"[tactile_processor] 【超阈值-平均】{name} 平均值 {avg:.1f} > {AVG_THRESHOLD}")
    for name, mx in maxs.items():
        if mx > MAX_THRESHOLD:
            rospy.loginfo(f"[tactile_processor] 【超阈值-最大】{name} 最大值 {mx:.1f} > {MAX_THRESHOLD}")


def cb_touch(msg: Int32MultiArray):
    data = list(msg.data)
    if len(data) == TOTAL_POINTS:
        # 每条消息正好一个周期：直接处理
        process_cycle(data)
    else:
        # 如果一条消息里包含多个周期或不足一个周期，按块拆
        offset = 0
        n = len(data)
        while offset + TOTAL_POINTS <= n:
            cycle = data[offset:offset+TOTAL_POINTS]
            process_cycle(cycle)
            offset += TOTAL_POINTS
        # if offset < n:
        #     rospy.logwarn(f"[tactile_processor] 丢弃尾部 {n-offset} 个点：对齐失败")
        

def main():
    rospy.init_node('tactile_processor', anonymous=True)
    rospy.loginfo("[tactile_processor] 节点已启动，订阅 /touch_data …")
    rospy.Subscriber('/touch_data', Int32MultiArray, cb_touch, queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    main()