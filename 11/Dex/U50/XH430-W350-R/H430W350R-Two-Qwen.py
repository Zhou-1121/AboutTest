#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
基于角度误差的闭环比例控制物体追踪示例

摄像头:Intel RealSense D455
舵机：两路 XH430-W350-R
思路：每帧根据目标像素误差计算角度误差，再按比例更新舵机角度。

双舵机测试脚本（各自独立 U2D2 → USB 口 + 不同波特率 + 同 ID=1)
- pan(水平) 舵机：   /dev/ttyUSB1 @ 1,000,000bps, ID=1
- tilt(垂直)舵机：  /dev/ttyUSB0 @  56,600bps, ID=1
"""

import sys, time
from Queen import get_center    # Qwen反馈模块，返回 cx, cy
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

# —————— 1. 配置区 ——————
# 串口 & 舵机 ID
PAN_PORT,  BAUD_PAN  = '/dev/ttyUSB1', 1_000_000
TILT_PORT, BAUD_TILT = '/dev/ttyUSB0', 57_600
PROTOCOL_VER         = 2.0
ID_PAN, ID_TILT      = 1, 1

# Control Table 地址 & 模式
ADDR_OP_MODE       = 11
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POS      = 116
ADDR_PRESENT_POS   = 132
OPER_MODE_POS      = 3
TORQUE_ON, TORQUE_OFF = 1, 0

# 零点脉冲 & 初始角度
PAN_CENTER_TICK,   TILT_CENTER_TICK= 2100, 1980
pan_angle_current, tilt_angle_current = 0.0, 0.0  # “中心”对应 0°

# 相机分辨率 & 视场角
IMAGE_WIDTH, IMAGE_HEIGHT = 1280, 720
H_FOV_DEG,  V_FOV_DEG     = 86.0, 57.0
# H_FOV_DEG,  V_FOV_DEG     = 70.0, 40.0

# 比例增益（0 < Kp ≤ 1）
Kp_pan, Kp_tilt = 0.7, 0.7

# —————— 2. 初始化各自端口 ——————
def init_port(dev, baud):
    ph = PortHandler(dev);              #ph串口句柄
    pk = PacketHandler(PROTOCOL_VER)    #pk协议句柄
    if not ph.openPort():   
        sys.exit(f"无法打开串口 {dev}")
    if not ph.setBaudRate(baud): 
        sys.exit(f"无法设置波特 {baud} on {dev}")
    return ph, pk

# —————— 模式切换 & 扭矩控制 ——————
def setup_servo(ph, pk, did):  #did是ID，addr寄存器位置       
    pk.write1ByteTxRx(ph, did, ADDR_OP_MODE,  OPER_MODE_POS)   # 切位置控制模式
    pk.write1ByteTxRx(ph, did, ADDR_TORQUE_ENABLE, TORQUE_ON)  # 使能扭矩  

# —————— 位置写入 & 读取 ——————
def write_goal(ph, pk, did, tick):
    pk.write4ByteTxRx(ph, did, ADDR_GOAL_POS, tick) #write1ByteTxRx写入一个字节（1byte）

def shutdown_servo(ph, pk, did):
    pk.write1ByteTxRx(ph, did, ADDR_TORQUE_ENABLE, TORQUE_OFF)
    ph.closePort()

# —————— 角度→脉冲映射 ——————
def deg2tick(angle_deg, center_tick):
    angle = max(-180.0, min(180.0, angle_deg))
    return int(center_tick + angle * 4096.0/360.0)

def pixel_to_error(cx, cy):
    """
    返回相对画面中心的角度误差 (pan_err, tilt_err)
    右偏 & 下偏 为正
    """
    dx = cx - IMAGE_WIDTH/2
    dy = cy - IMAGE_HEIGHT/2
    pan_err  = dx / (IMAGE_WIDTH/2)  * (H_FOV_DEG/2)
    tilt_err = -dy/ (IMAGE_HEIGHT/2) * (V_FOV_DEG/2)
    return pan_err, tilt_err

# —————— 3. 主流程 ——————
def main():
    # a) 初始化两路串口 & 舵机
    ph_pan,  pk_pan  = init_port(PAN_PORT,  BAUD_PAN)
    ph_tilt, pk_tilt = init_port(TILT_PORT, BAUD_TILT)
    setup_servo(ph_pan,  pk_pan,  ID_PAN)
    setup_servo(ph_tilt, pk_tilt, ID_TILT)

    global pan_angle_current, tilt_angle_current
    try:
        while True:
            # 1) 取中心点
            cx, cy = get_center()  
            # 2) 计算角度误差
            pan_err, tilt_err = pixel_to_error(cx, cy)

            # 3) 按比例更新目标角度
            pan_angle_current  += Kp_pan  * pan_err
            tilt_angle_current += Kp_tilt * tilt_err

            # 限幅
            pan_angle_current  = max(-180, min(180, pan_angle_current))
            tilt_angle_current = max(-180, min(180, tilt_angle_current))

            # 4) 映射 & 下发
            pan_tick  = deg2tick(pan_angle_current,  PAN_CENTER_TICK)
            tilt_tick = deg2tick(tilt_angle_current, TILT_CENTER_TICK)
            write_goal(ph_pan,  pk_pan,  ID_PAN,  pan_tick)
            write_goal(ph_tilt, pk_tilt, ID_TILT, tilt_tick)

            # 调试输出
            print(f"err°=({pan_err:.2f},{tilt_err:.2f})  ang°=({pan_angle_current:.2f},{tilt_angle_current:.2f})")

            time.sleep(0.03)
    except KeyboardInterrupt:
        pass
    finally:
        shutdown_servo(ph_pan,  pk_pan,  ID_PAN)
        shutdown_servo(ph_tilt, pk_tilt, ID_TILT)

if __name__ == '__main__':
    main()