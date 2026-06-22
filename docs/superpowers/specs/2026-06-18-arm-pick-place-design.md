# 机械臂抓取与递送系统设计文档

## 项目概述

**项目名称**：机器人机械臂抓取与递送系统

**项目目标**：控制移动机器人搭载的多自由度机械臂，结合视觉定位，完成目标物体抓取、定点递送、放置复位整套流程，模拟小型物料搬运、物品递送作业。

**技术栈**：
- 开发框架：ROS2 (Galactic+)
- 仿真环境：Gazebo Classic
- 视觉处理：OpenCV 4.5+
- 机械臂模型：WPR系列仿真机器人
- 目标物体：简单几何体（立方体、圆柱体）

## 系统架构

### 架构图

```
┌─────────────────────────────────────────────────────────────────┐
│                        Gazebo仿真环境                           │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐          │
│  │ WPR机器人模型 │  │ 目标物体     │  │ 工作台面     │          │
│  └──────────────┘  └──────────────┘  └──────────────┘          │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                        ROS2通信层                               │
│  ┌────────────┐ ┌────────────┐ ┌────────────┐ ┌────────────┐   │
│  │ /camera/   │ │ /arm/      │ │ /gripper/  │ │ /task/     │   │
│  │ image      │ │ joint_states│ │ control    │ │ status     │   │
│  └────────────┘ └────────────┘ └────────────┘ └────────────┘   │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                        ROS2节点层                               │
│  ┌────────────┐ ┌────────────┐ ┌────────────┐ ┌────────────┐   │
│  │ 视觉识别节点│ │ 机械臂控制 │ │ 抓取逻辑节点│ │ 任务调度节点│   │
│  │ (vision)   │ │ 节点(arm)  │ │ (gripper)  │ │ (scheduler)│   │
│  └────────────┘ └────────────┘ └────────────┘ └────────────┘   │
└─────────────────────────────────────────────────────────────────┘
```

### 核心节点职责

| 节点名称 | 职责 | 订阅话题 | 发布话题 |
|---------|------|---------|---------|
| `vision_node` | 目标检测与定位 | `/camera/image_raw` | `/vision/target_pose` |
| `arm_controller` | 机械臂关节控制 | `/vision/target_pose`, `/task/command` | `/arm/joint_states`, `/task/status` |
| `gripper_controller` | 夹爪控制 | `/task/command` | `/gripper/state` |
| `task_scheduler` | 任务流程调度 | `/arm/joint_states`, `/gripper/state`, `/vision/target_pose` | `/task/command`, `/task/status` |

## 数据流设计

### 话题定义

```
话题1: /camera/image_raw
├── 类型: sensor_msgs/msg/Image
├── 频率: 30Hz
└── 描述: 摄像头原始图像流

话题2: /vision/target_pose
├── 类型: geometry_msgs/msg/PoseStamped
├── 频率: 10Hz (检测到目标时发布)
└── 描述: 目标物体的位姿信息
    ├── position: x, y, z (相对于机器人基座)
    └── orientation: 四元数表示

话题3: /task/command
├── 类型: 自定义消息 TaskCommand
├── 频率: 事件触发
└── 描述: 任务指令
    ├── command_type: GRASP / DELIVER / RESET
    └── target_pose: 目标位姿

话题4: /task/status
├── 类型: 自定义消息 TaskStatus
├── 频率: 事件触发
└── 描述: 任务状态反馈
    ├── state: IDLE / MOVING / GRASPING / DELIVERING / ERROR
    └── progress: 0.0 ~ 1.0
```

### 自定义消息文件

```msg
# TaskCommand.msg
uint8 command_type
uint8 GRASP=1
uint8 DELIVER=2
uint8 RESET=3
geometry_msgs/PoseStamped target_pose
---

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

## 节点详细设计

### 1. 视觉识别节点 (vision_node)

**功能**：
- 订阅摄像头图像
- 使用OpenCV进行目标检测
- 计算目标物体的3D位姿
- 发布目标位姿到话题

**核心算法**：
```python
# 目标检测流程
1. 图像预处理 (去噪、色彩空间转换)
2. 轮廓检测 (阈值分割)
3. 形状筛选 (面积、周长、圆形度)
4. 位姿估计 (相机内参 + 深度信息)
5. 坐标变换 (相机坐标系 → 机器人基座坐标系)
```

**参数配置**：
```yaml
vision_node:
  ros__parameters:
    target_color: "red"           # 目标颜色
    min_area: 500                 # 最小面积阈值
    max_area: 50000               # 最大面积阈值
    camera_frame: "camera_link"   # 相机坐标系
```

### 2. 机械臂控制节点 (arm_controller)

**功能**：
- 接收目标位姿
- 逆运动学求解
- 控制关节运动
- 反馈当前状态

**运动规划**：
```python
# 运动流程
1. 逆运动学求解 (目标位姿 → 关节角度)
2. 轨迹规划 (生成平滑轨迹)
3. 关节控制 (发布关节指令)
4. 状态监控 (实时反馈)
```

**参数配置**：
```yaml
arm_controller:
  ros__parameters:
    arm_joint_names: ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
    max_velocity: 0.5             # 最大角速度 (rad/s)
    max_acceleration: 1.0         # 最大角加速度 (rad/s²)
    home_position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 零位
```

### 3. 夹爪控制节点 (gripper_controller)

**功能**：
- 接收抓取/释放指令
- 控制夹爪开合
- 反馈夹爪状态

**状态机**：
```
IDLE → OPEN → CLOSING → CLOSED → OPENING → OPEN
         ↑                                    │
         └────────────────────────────────────┘
```

**参数配置**：
```yaml
gripper_controller:
  ros__parameters:
    open_position: 0.04           # 夹爪全开位置 (m)
    close_position: 0.0           # 夹爪全闭位置 (m)
    close_force: 20.0             # 夹持力 (N)
    action_timeout: 5.0           # 动作超时 (s)
```

### 4. 任务调度节点 (task_scheduler)

**功能**：
- 管理整个抓取-递送流程
- 协调各节点工作
- 处理异常情况

**状态机**：
```
IDLE → DETECTING → MOVING_TO_OBJECT → GRASPING → 
      MOVING_TO_TARGET → RELEASING → RETURNING → IDLE
```

**流程控制**：
```python
# 完整流程
1. 初始化: 所有节点归零位
2. 检测: 视觉节点寻找目标
3. 移动: 机械臂移动到目标上方
4. 下降: 机械臂下降到抓取高度
5. 抓取: 夹爪闭合
6. 提升: 机械臂提升
7. 移动: 机械臂移动到递送点
8. 下降: 机械臂下降到放置高度
9. 释放: 夹爪打开
10. 复位: 机械臂返回零位
```

## 项目结构

```
arm_pick_and_place/
├── src/
│   ├── vision_node/
│   │   ├── vision_node/
│   │   │   ├── __init__.py
│   │   │   └── vision_node.py
│   │   ├── launch/
│   │   │   └── vision.launch.py
│   │   ├── package.xml
│   │   ├── setup.py
│   │   └── setup.cfg
│   │
│   ├── arm_controller/
│   │   ├── arm_controller/
│   │   │   ├── __init__.py
│   │   │   ├── arm_controller.py
│   │   │   └── ik_solver.py
│   │   ├── launch/
│   │   │   └── arm.launch.py
│   │   ├── package.xml
│   │   ├── setup.py
│   │   └── setup.cfg
│   │
│   ├── gripper_controller/
│   │   ├── gripper_controller/
│   │   │   ├── __init__.py
│   │   │   └── gripper_controller.py
│   │   ├── launch/
│   │   │   └── gripper.launch.py
│   │   ├── package.xml
│   │   ├── setup.py
│   │   └── setup.cfg
│   │
│   └── task_scheduler/
│       ├── task_scheduler/
│       │   ├── __init__.py
│       │   ├── task_scheduler.py
│       │   └── states.py
│       ├── launch/
│       │   └── task.launch.py
│       ├── package.xml
│       ├── setup.py
│       └── setup.cfg
│
├── msg/
│   ├── TaskCommand.msg
│   └── TaskStatus.msg
│
├── urdf/
│   └── wpr_arm.urdf.xacro
│
├── worlds/
│   └── pick_place_world.sdf
│
├── config/
│   └── params.yaml
│
├── launch/
│   └── full_system.launch.py
│
└── README.md
```

## 依赖列表

| 依赖包 | 用途 | 版本要求 |
|-------|------|---------|
| `rclpy` | ROS2 Python客户端 | Galactic+ |
| `sensor_msgs` | 传感器消息 | Galactic+ |
| `geometry_msgs` | 几何消息 | Galactic+ |
| `std_msgs` | 标准消息 | Galactic+ |
| `opencv-python` | 图像处理 | 4.5+ |
| `numpy` | 数值计算 | 1.20+ |
| `moveit` | 运动规划 | Galactic+ |
| `tf2_ros` | 坐标变换 | Galactic+ |

## 启动流程

### 主启动文件

```python
# launch/full_system.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 启动Gazebo仿真环境
        Node(
            package='gazebo_ros',
            executable='gazebo',
            arguments=['-world', 'pick_place_world.sdf']
        ),
        
        # 启动视觉节点
        Node(
            package='vision_node',
            executable='vision_node',
            name='vision_node',
            parameters=['config/params.yaml']
        ),
        
        # 启动机械臂控制节点
        Node(
            package='arm_controller',
            executable='arm_controller',
            name='arm_controller',
            parameters=['config/params.yaml']
        ),
        
        # 启动夹爪控制节点
        Node(
            package='gripper_controller',
            executable='gripper_controller',
            name='gripper_controller',
            parameters=['config/params.yaml']
        ),
        
        # 启动任务调度节点
        Node(
            package='task_scheduler',
            executable='task_scheduler',
            name='task_scheduler',
            parameters=['config/params.yaml']
        ),
    ])
```

### 参数配置文件

```yaml
# config/params.yaml
vision_node:
  ros__parameters:
    target_color: "red"
    min_area: 500
    max_area: 50000
    camera_frame: "camera_link"
    detection_threshold: 0.5

arm_controller:
  ros__parameters:
    arm_joint_names: 
      - "joint1"
      - "joint2"
      - "joint3"
      - "joint4"
      - "joint5"
      - "joint6"
    max_velocity: 0.5
    max_acceleration: 1.0
    home_position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    pick_height: 0.1
    place_height: 0.1

gripper_controller:
  ros__parameters:
    open_position: 0.04
    close_position: 0.0
    close_force: 20.0
    action_timeout: 5.0

task_scheduler:
  ros__parameters:
    pick_position: [0.3, 0.0, 0.1]
    place_position: [-0.3, 0.0, 0.1]
    safe_height: 0.3
    move_timeout: 10.0
```

## 运行命令

```bash
# 1. 编译工作空间
cd ~/ros2_ws
colcon build
source install/setup.bash

# 2. 启动完整系统
ros2 launch arm_pick_and_place full_system.launch.py

# 3. 触发抓取任务 (使用自定义消息)
ros2 topic pub --once /task/command arm_pick_and_place/msg/TaskCommand "{command_type: 1}"

# 4. 监控任务状态
ros2 topic echo /task/status
```
