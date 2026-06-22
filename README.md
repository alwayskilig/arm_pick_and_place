# 机械臂抓取与递送系统

基于ROS2的机械臂抓取与递送系统，包含视觉识别、机械臂控制、夹爪控制和任务调度四个核心模块。

## 功能特性

- 视觉识别：使用OpenCV进行目标物体检测和定位
- 机械臂控制：逆运动学求解和关节控制
- 夹爪控制：开合控制和状态反馈
- 任务调度：完整的抓取-递送流程管理

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
    ros-galactic-rclpy \
    ros-galactic-sensor-msgs \
    ros-galactic-geometry-msgs \
    ros-galactic-std-msgs \
    ros-galactic-tf2-ros \
    ros-galactic-cv-bridge

# 安装Python依赖
pip install opencv-python numpy
```

## 编译

```bash
cd ~/ros2_ws
colcon build --packages-select arm_pick_and_place vision_node arm_controller gripper_controller task_scheduler
source install/setup.bash
```

## 运行

```bash
# 启动完整系统
ros2 launch arm_pick_and_place full_system.launch.py

# 监控任务状态
ros2 topic echo /task/status
```

## 话题说明

| 话题名称 | 类型 | 描述 |
|---------|------|------|
| `/camera/image_raw` | sensor_msgs/Image | 摄像头图像流 |
| `/vision/target_pose` | geometry_msgs/PoseStamped | 目标物体位姿 |
| `/task/command` | arm_pick_and_place/TaskCommand | 任务指令 |
| `/task/status` | arm_pick_and_place/TaskStatus | 任务状态 |
| `/arm/joint_states` | sensor_msgs/JointState | 关节状态 |
| `/gripper/state` | std_msgs/Float64 | 夹爪状态 |

## 参数配置

所有参数配置在 `config/params.yaml` 文件中。

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
