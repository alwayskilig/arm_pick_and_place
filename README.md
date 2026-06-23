# 机械臂抓取与递送系统

基于ROS2的机械臂抓取与递送系统，包含视觉识别、机械臂控制、夹爪控制和任务调度四个核心模块。

## 功能特性

- 视觉识别：使用OpenCV进行目标物体检测和定位
- 机械臂控制：逆运动学求解和关节控制
- 夹爪控制：开合控制和状态反馈
- 任务调度：完整的抓取-递送流程管理
- **双模式支持**：同时支持Gazebo仿真和PadBot E2真实硬件

## 支持模式

### 1. 仿真模式（当前默认）
- **环境**：Gazebo仿真，无需真实硬件
- **云台控制**：`PoseStamped` 消息，话题 `/pan_tilt/command`
- **机械臂**：仿真逆运动学，通过 `arm_controller` 节点控制
- **启动文件**：`full_system.launch.py`

### 2. 硬件模式（PadBot E2）
- **环境**：真实机器人，需要Dobot机械臂、云台、摄像头
- **云台控制**：`PanTiltControl` 消息，话题 `/pan_tilt_control`
- **机械臂**：Dobot SDK，通过 `dobot_controller` 节点控制
- **启动文件**：`full_system_hardware.launch.py`（需创建）
- **配置指南**：参阅 `HARDWARE_INTEGRATION.md`（需创建）

## 系统架构

```
┌─────────────────────────────────────────────────────────────────┐
│                        Gazebo仿真环境                           │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                        ROS2通信层                               │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│  ┌────────────┐ ┌────────────┐ ┌────────────┐ ┌────────────┐   │
│  │ 视觉识别节点│ │ 机械臂控制 │ │ 夹爪控制节点│ │ 任务调度节点│   │
│  └────────────┘ └────────────┘ └────────────┘ └────────────┘   │
└─────────────────────────────────────────────────────────────────┘
```

## 安装依赖

```bash
# 安装ROS2依赖
sudo apt-get install -y \
    ros-humble-rclpy \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-std-msgs \
    ros-humble-tf2-ros \
    ros-humble-cv-bridge

# 安装Python依赖
pip install opencv-python numpy
```

## 编译

```bash
cd ~/ros2_ws

# 编译所有包（含子包）
colcon build

# 或者只编译需要的包
colcon build --packages-select arm_pick_and_place vision_node arm_controller gripper_controller task_scheduler

source install/setup.bash
```

## 运行

### 仿真模式（Gazebo）
```bash
# 启动仿真系统
ros2 launch arm_pick_and_place full_system.launch.py

# 监控任务状态
ros2 topic echo /task/status
```

### 硬件模式（PadBot E2）
```bash
# 1. 启动硬件驱动
ros2 run padbot_robot_arm dobot_controller
ros2 run padbot_pan_tilt pan_tilt_node

# 2. 启动硬件系统（需创建启动文件）
ros2 launch arm_pick_and_place full_system_hardware.launch.py

# 或手动启动各节点
```

## 话题说明

### 通用话题（仿真和硬件模式）
| 话题名称 | 类型 | 描述 |
|---------|------|------|
| `/camera/image_raw` | sensor_msgs/Image | 摄像头图像流 |
| `/vision/target_pose` | geometry_msgs/PoseStamped | 目标物体位姿 |
| `/task/command` | arm_pick_and_place/TaskCommand | 任务指令 |
| `/task/status` | arm_pick_and_place/TaskStatus | 任务状态 |
| `/arm/joint_states` | sensor_msgs/JointState | 关节状态 |
| `/gripper/state` | std_msgs/Float64 | 夹爪状态 |

### 硬件模式特有话题
| 话题名称 | 类型 | 描述 |
|---------|------|------|
| `/arm/target_pose` | geometry_msgs/PoseStamped | 机械臂目标位姿（直通Dobot） |
| `/pan_tilt_control` | padbot_pan_tilt/PanTiltControl | 云台控制指令 |
| `/pan_tilt/state` | padbot_pan_tilt/PanTiltInfo | 云台状态信息 |

## 参数配置

所有参数配置在 `config/params.yaml` 文件中。

### 仿真模式参数（默认）
```yaml
vision_node:
  ros__parameters:
    camera_topic: "/camera/image"  # 仿真摄像头话题
    
arm_controller:
  ros__parameters:
    use_hardware: false  # 使用仿真逆运动学
```

### 硬件模式参数（需修改）
```yaml
vision_node:
  ros__parameters:
    camera_topic: "/camera/image_raw"  # 实际摄像头话题
    
arm_controller:
  ros__parameters:
    use_hardware: true  # 使用Dobot硬件
```

## 系统测试与验证

### 快速测试

```bash
# 1. 编译消息
colcon build --packages-select arm_pick_and_place
source install/setup.bash

# 2. 验证消息定义
ros2 interface show arm_pick_and_place/msg/TaskCommand
ros2 interface show arm_pick_and_place/msg/TaskStatus

# 3. 启动完整系统
ros2 launch arm_pick_and_place full_system.launch.py

# 4. 监控话题
ros2 topic list
ros2 topic echo /task/status
```

### 发送测试指令

```bash
# 发送RESET指令（复位）
ros2 topic pub --once /task/command arm_pick_and_place/msg/TaskCommand "{command_type: 3}"

# 发送GRASP指令（抓取）
ros2 topic pub --once /task/command arm_pick_and_place/msg/TaskCommand "{command_type: 1}"

# 发送DELIVER指令（递送）
ros2 topic pub --once /task/command arm_pick_and_place/msg/TaskCommand "{command_type: 2}"
```

### 自动化测试

```bash
python3 test/test_system.py
```

详细测试文档请参阅 `test/TESTING.md`。

## 硬件集成（PadBot E2）

### 准备工作
1. 参阅 `HARDWARE_INTEGRATION.md`（需创建）
2. 修改 `padbot_E2` 中的 `dobot.cpp` 文件
3. 编译硬件驱动包

### 硬件测试步骤
```bash
# 1. 编译硬件版本
cd ~/ros2_ws
colcon build --packages-select arm_pick_and_place padbot_robot_arm padbot_pan_tilt
source install/setup.bash

# 2. 启动硬件驱动
ros2 run padbot_robot_arm dobot_controller
ros2 run padbot_pan_tilt pan_tilt_node

# 3. 启动系统
ros2 launch arm_pick_and_place full_system.launch.py

# 4. 测试硬件
ros2 topic pub --once /task/command arm_pick_and_place/msg/TaskCommand "{command_type: 1}"
```

### 注意事项
- 确保Dobot机械臂已连接并上电
- 摄像头需要正确校准
- 首次使用需校准机械臂零位
- 调试时建议使用低速模式
