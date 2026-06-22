# 系统测试与验证文档

## 概述

本文档描述了对ROS2机械臂抓取与递送系统的完整测试流程，包括消息编译验证、节点启动测试和话题通信测试。

## 测试环境要求

- ROS2 (Galactic/Humble)
- Ubuntu 20.04/22.04
- OpenCV 4.x
- Python 3.8+

## 前置步骤

### 1. 编译所有包

```bash
cd ~/ros2_ws
colcon build --packages-select arm_pick_and_place vision_node arm_controller gripper_controller task_scheduler
source install/setup.bash
```

### 2. 验证编译结果

```bash
# 验证消息编译
ros2 interface show arm_pick_and_place/msg/TaskCommand
ros2 interface show arm_pick_and_place/msg/TaskStatus
```

预期输出：
```
# TaskCommand.msg
uint8 command_type
uint8 GRASP=1
uint8 DELIVER=2
uint8 RESET=3
geometry_msgs/PoseStamped target_pose
```

```
# TaskStatus.msg
uint8 state
uint8 IDLE=0
uint8 MOVING=1
uint8 GRASPING=2
uint8 DELIVERING=3
uint8 ERROR=4
float32 progress
string message
```

---

## 测试项目

### 测试1: 消息编译验证

**目的**: 验证自定义消息类型正确编译

**步骤**:
```bash
cd arm_pick_and_place
colcon build --packages-select arm_pick_and_place
ros2 interface show arm_pick_and_place/msg/TaskCommand
ros2 interface show arm_pick_and_place/msg/TaskStatus
```

**预期结果**: 显示完整的消息定义

---

### 测试2: 节点启动验证

**目的**: 验证所有节点可以正常启动

**方法A - 逐个启动**:
```bash
# 终端1: 启动视觉节点
ros2 run vision_node vision_node

# 终端2: 启动机械臂控制节点
ros2 run arm_controller arm_controller

# 终端3: 启动夹爪控制节点
ros2 run gripper_controller gripper_controller

# 终端4: 启动任务调度节点
ros2 run task_scheduler task_scheduler
```

**方法B - 使用launch文件**:
```bash
# 启动完整系统
ros2 launch arm_pick_and_place full_system.launch.py
```

**预期结果**: 各节点正常启动，日志显示"initialized"信息，无错误

---

### 测试3: 话题通信验证

**目的**: 验证节点间可以正常通信

**步骤**:
```bash
# 终端1: 列出所有话题
ros2 topic list

# 终端2: 监听任务状态
ros2 topic echo /task/status

# 终端3: 发送测试指令
ros2 topic pub --once /task/command arm_pick_and_place/msg/TaskCommand "{command_type: 3}"
```

**预期话题列表**:
```
/arm/joint_states
/camera/image_raw
/gripper/state
/task/command
/task/status
/vision/target_pose
```

**预期结果**: 能正常发送和接收消息

---

### 测试4: 夹爪控制验证

**目的**: 验证夹爪控制器响应指令

**步骤**:
```bash
# 发送GRASP指令（关闭夹爪）
ros2 topic pub --once /task/command arm_pick_and_place/msg/TaskCommand "{command_type: 1}"

# 监听夹爪状态
ros2 topic echo /gripper/state

# 发送DELIVER指令（打开夹爪）
ros2 topic pub --once /task/command arm_pick_and_place/msg/TaskCommand "{command_type: 2}"
```

**预期结果**: 夹爪状态值从0.04（打开）变为0.0（关闭）再变回0.04

---

### 测试5: Launch文件验证

**目的**: 验证launch文件语法正确

**步骤**:
```bash
ros2 launch arm_pick_and_place full_system.launch.py --show-args
```

**预期结果**: 显示launch文件参数信息，无语法错误

---

## 自动化测试

使用提供的自动化测试脚本：

```bash
cd arm_pick_and_place
python3 test/test_system.py
```

脚本将自动执行所有测试项目并输出结果汇总。

---

## 测试结果记录

| 测试项目 | 状态 | 备注 |
|---------|------|------|
| 消息编译验证 | ☐ | |
| 节点启动验证 | ☐ | |
| 话题通信验证 | ☐ | |
| 夹爪控制验证 | ☐ | |
| Launch文件验证 | ☐ | |

---

## 常见问题排查

### 1. 编译失败
- 检查ROS2环境是否正确source
- 检查依赖包是否安装完整

### 2. 节点启动失败
- 检查Python依赖是否安装: `pip install opencv-python numpy`
- 检查cv_bridge是否安装: `sudo apt-get install ros-galactic-cv-bridge`

### 3. 话题通信失败
- 检查节点是否正常运行: `ros2 node list`
- 检查话题是否正确发布: `ros2 topic info /task/status`

### 4. 夹爪控制无响应
- 检查/gripper/state话题是否有数据
- 检查/task/command话题是否正确发布

---

## 系统话题说明

| 话题名称 | 类型 | 方向 | 描述 |
|---------|------|------|------|
| `/camera/image_raw` | sensor_msgs/Image | 发布 | 摄像头图像流 |
| `/vision/target_pose` | geometry_msgs/PoseStamped | 发布 | 目标物体位姿 |
| `/task/command` | arm_pick_and_place/TaskCommand | 订阅 | 任务指令 |
| `/task/status` | arm_pick_and_place/TaskStatus | 发布 | 任务状态 |
| `/arm/joint_states` | sensor_msgs/JointState | 订阅/发布 | 关节状态 |
| `/gripper/state` | std_msgs/Float64 | 发布 | 夹爪状态 |

---

## 指令类型说明

| 指令类型 | 值 | 描述 |
|---------|---|------|
| GRASP | 1 | 抓取（关闭夹爪） |
| DELIVER | 2 | 递送（打开夹爪） |
| RESET | 3 | 复位（返回初始位置） |

---

## 状态码说明

| 状态 | 值 | 描述 |
|------|---|------|
| IDLE | 0 | 空闲 |
| MOVING | 1 | 移动中 |
| GRASPING | 2 | 抓取中 |
| DELIVERING | 3 | 递送中 |
| ERROR | 4 | 错误 |
