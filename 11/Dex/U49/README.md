<h1 align="center"> SRCB DexGraspVLA Project </h1>

# Robot Environment Bringup

### First, Run Robot Body:
```bash
ssh nvidia@192.168.3.150 (password: nvidia)
cd ~/atc_standard-V1.1.8/install/share/startup_config/script
./robot_startup.sh boot ../session.d/ATCStandard/SRCBBringup.d
```
### Run Robot Hand:

```bash
ssh samsung-Alienware-Area-51m (password: 123456)
cd yixiang/inspire_hand_ws/src/inspire_hand_modbusTCP/build/
./hand_modbus_control
```

```bash
source robot-controller/inspire_hand_ws/devel/setup.bash
```
### Run VR:

```bash
ssh nvidia@192.168.3.150 (password: nvidia)

cd ~/atc_standard-V1.1.8/install/share/startup_config/script

ROS_IP=192.168.3.150 ROS_MASTER_URI=http://192.168.3.150:11311 VR_IP=192.168.3.52 ./robot_startup.sh boot ../session.d/ATCStandard/R1PROVRTeleop.d/
```
```bash
python3 /data/yixiang/SRCB-DexVLA/Inspired_hand_vr_utils.py

ssh nvidia@192.168.3.150 (password: nvidia)

rosbag record -O /home/dataset/hand_$(date +%Y%m%d%H%M%S%3N)_RAW /hand/angle_actual
```
```bash
ssh nvidia@192.168.3.150 (password: nvidia)

rosbag record -O ~/GalaxeaDataset/hand_$(date +%Y%m%d%H%M%S%3N)_RAW /hand/angle_actual
```
### Run DexGraspVLA

```bash
python3 inference_v3.py --use_omni
```

### Run Robot Arm
In [Galaxea_R1_interface.py](/Galaxea_R1_Interface.py), use ```move_ee(self, position, orientation, euler_mode=False, right_arm=True):``` can move robot EE. ```right_arm=True``` to use right arm and ```right_arm=False``` to tus left arm.

eg.
``
robot.move_ee([0,-0.2521,-0.4016], [0, 0, 0, 1])
``

![](/temp/FrameEE.PNG)
注意：当前EE是相对于腰部坐标系的，但是当腰移动后，坐标系也不会变，是固定坐标系。后续基于底盘base坐标系？

相关代码在nvidia板卡的: /home/nvidia/atc_standard-V1.1.8/install/lib/mobiman/configs/R1PRO/settings_left.yaml。这里可以设置base坐标系。

TODO: 做好坐标系转换,将头部相机坐标系转换到和胳膊转到相同坐标系下。
# IP
nvidia: 192.168.3.150

hand: 192.168.3.44

Alienware: 192.168.3.11

Workstation: 192.168.3.100

VR:192.168.3.52