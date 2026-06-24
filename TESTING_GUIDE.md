# 机械臂抓取系统仿真测试指南

## 概述

本文档提供完整的仿真测试流程，用于验证机械臂抓取与递送系统在 Gazebo 仿真环境中的功能。

**目标：** 在 WPR 仿真环境中测试完整的抓取-递送流程

**系统要求：**
- ROS2 Humble
- Gazebo Classic
- wpr_simulation2 包
- arm_pick_and_place 包

---

## 测试前准备

### 1. 环境检查

```bash
# 检查 ROS2 环境
echo $ROS_DISTRO
# 应输出: humble

# 检查 Gazebo
gazebo --version
# 应显示版本信息

# 检查 wpr_simulation2
ros2 pkg list | grep wpr_simulation2
```

### 2. 编译项目

```bash
cd ~/ros2_ws

# 编译所有包
colcon build

# 或者只编译需要的包
colcon build --packages-select arm_pick_and_place

source install/setup.bash
```

### 3. 验证编译结果

```bash
# 检查包是否安装成功
ros2 pkg list | grep arm_pick_and_place

# 检查可执行文件
ros2 pkg executables arm_pick_and_place
```

---

## 测试流程

### 阶段一：启动仿真环境

**终端1：启动 Gazebo 仿真**

```bash
cd ~/ros2_ws
source install/setup.bash

# 启动带机械臂的仿真环境
ros2 launch wpr_simulation2 wpb_mani.launch.py
```

**预期结果：**
- Gazebo 窗口打开
- 显示 WPR 机器人（带机械臂）
- 无报错信息

**验证命令：**
```bash
# 在新终端检查话题
ros2 topic list | grep wpb_home
# 应看到: /wpb_home/mani_ctrl
```

### 阶段二：启动任务节点

**终端2：启动 arm_pick_and_place 节点**

```bash
cd ~/ros2_ws
source install/setup.bash

ros2 launch arm_pick_and_place full_system.launch.py
```

**预期结果：**
- 看到以下节点启动信息：
  - `vision_node initialized`
  - `Arm controller initialized (simulation mode)`
  - `Gripper controller initialized`
  - `Task scheduler initialized`

**验证命令：**
```bash
# 检查节点是否运行
ros2 node list
# 应看到: /vision_node, /arm_controller, /gripper_controller, /task_scheduler

# 检查话题
ros2 topic list
```

### 阶段三：手动测试各模块

#### 测试1：视觉节点

```bash
# 发布测试图像（如果没有摄像头）
ros2 run rqt_image_view rqt_image_view /camera/image

# 或者检查视觉节点是否订阅正确
ros2 topic info /camera/image
```

#### 测试2：云台控制

```bash
# 检查云台话题
ros2 topic list | grep pan_tilt

# 发送云台控制指令（如果需要）
ros2 topic pub --once /pan_tilt/command geometry_msgs/msg/PoseStamped "{header: {stamp: {sec: 0}, frame_id: 'base_link'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```

#### 测试3：机械臂控制

```bash
# 检查机械臂话题
ros2 topic list | grep arm

# 发送目标位姿测试
ros2 topic pub --once /vision/target_pose geometry_msgs/msg/PoseStamped "{header: {stamp: {sec: 0}, frame_id: 'base_link'}, pose: {position: {x: 0.3, y: 0.0, z: 0.5}, orientation: {w: 1.0}}}"
```

#### 测试4：夹爪控制

```bash
# 检查夹爪状态
ros2 topic echo /gripper/state
```

### 阶段四：完整流程测试

#### 测试A：复位指令

```bash
ros2 topic pub --once /task/command arm_pick_and_place/msg/TaskCommand "{command_type: 3}"
```

**预期结果：**
- 机械臂移动到初始位置
- 状态显示 `Returning to home`

#### 测试B：抓取指令

```bash
# 先确保有目标物体在摄像头视野中
ros2 topic pub --once /task/command arm_pick_and_place/msg/TaskCommand "{command_type: 1}"
```

**预期结果：**
- 状态从 `DETECTING` → `MOVING_TO_OBJECT` → `GRASPING` → `LIFTING`
- 机械臂移动到目标位置
- 夹爪闭合

#### 测试C：递送指令

```bash
ros2 topic pub --once /task/command arm_pick_and_place/msg/TaskCommand "{command_type: 2}"
```

**预期结果：**
- 状态从 `MOVING_TO_TARGET` → `RELEASING` → `RETURNING`
- 机械臂移动到放置位置
- 夹爪打开

### 阶段五：监控与调试

#### 监控话题

```bash
# 监控任务状态
ros2 topic echo /task/status

# 监控关节状态
ros2 topic echo /joint_states

# 监控目标到达状态
ros2 topic echo /arm/goal_reached
```

#### 查看节点日志

```bash
# 查看所有节点日志
ros2 node info /arm_controller
ros2 node info /task_scheduler
```

#### 常见问题排查

| 问题 | 可能原因 | 解决方法 |
|------|----------|----------|
| Gazebo 打不开 | 依赖未安装 | 运行 `install_for_humble.sh` |
| 节点未找到 | 未编译 | 重新 `colcon build` |
| 话题无响应 | 节点未启动 | 检查终端2的输出 |
| 机械臂不动 | 仿真未启动 | 确认终端1的 Gazebo 正常 |

---

## 完整测试脚本

创建一个测试脚本 `test_simulation.sh`：

```bash
#!/bin/bash

echo "=== 机械臂抓取系统仿真测试 ==="

# 检查 ROS2 环境
if [ -z "$ROS_DISTRO" ]; then
    echo "错误: ROS2 环境未设置"
    echo "请运行: source /opt/ros/humble/setup.bash"
    exit 1
fi

echo "1. 编译项目..."
cd ~/ros2_ws
colcon build --packages-select arm_pick_and_place
source install/setup.bash

echo "2. 启动仿真环境（请在新终端运行）"
echo "命令: ros2 launch wpr_simulation2 wpb_mani.launch.py"
read -p "按回车继续..."

echo "3. 启动任务节点（请在新终端运行）"
echo "命令: ros2 launch arm_pick_and_place full_system.launch.py"
read -p "按回车继续..."

echo "4. 发送测试指令"
echo "发送复位指令..."
ros2 topic pub --once /task/command arm_pick_and_place/msg/TaskCommand "{command_type: 3}"

echo "等待5秒..."
sleep 5

echo "发送抓取指令..."
ros2 topic pub --once /task/command arm_pick_and_place/msg/TaskCommand "{command_type: 1}"

echo "=== 测试完成 ==="
echo "请观察 Gazebo 中的机器人动作"
```

运行脚本：
```bash
chmod +x test_simulation.sh
./test_simulation.sh
```

---

## 预期输出示例

### 正常启动输出

```
[INFO] [launch]: All log files can be found below /home/user/.ros/log/...
[INFO] [vision_node]: Vision node initialized, subscribing to: /camera/image
[INFO] [arm_controller]: Arm controller initialized (simulation mode)
[INFO] [gripper_controller]: Gripper controller initialized
[INFO] [task_scheduler]: Task scheduler initialized
[INFO] [task_scheduler]: Starting pick and place task
```

### 任务执行输出

```
[INFO] [task_scheduler]: Target detected: position=(0.3, 0.0, 0.5)
[INFO] [task_scheduler]: Target found, moving to object
[INFO] [arm_controller]: Received target pose: position=(0.3, 0.0, 0.5)
[INFO] [task_scheduler]: Goal reached signal received
[INFO] [task_scheduler]: Reached object, starting grasp
[INFO] [task_scheduler]: Grasp complete, lifting
[INFO] [task_scheduler]: Lift complete, moving to target
[INFO] [task_scheduler]: Reached target, releasing
[INFO] [task_scheduler]: Release complete, returning
[INFO] [task_scheduler]: Task complete
```

---

## 测试检查清单

- [ ] Gazebo 仿真环境正常启动
- [ ] 机器人模型正确显示
- [ ] 所有节点正常启动
- [ ] 话题通信正常
- [ ] 视觉节点能检测目标
- [ ] 机械臂能移动到目标位置
- [ ] 夹爪能正确开合
- [ ] 任务状态流转正常
- [ ] 完整流程能执行完成

---

## 故障排除

### 问题1：colcon build 失败

```bash
# 清理后重新编译
cd ~/ros2_ws
rm -rf build/ install/ log/
colcon build
```

### 问题2：Gazebo 启动失败

```bash
# 检查 Gazebo 依赖
sudo apt-get install -y ros-humble-gazebo-ros-pkgs
sudo apt-get install -y ros-humble-gazebo-plugins
```

### 问题3：节点找不到

```bash
# 确认包已安装
ros2 pkg list | grep arm_pick_and_place

# 重新 source 环境
source ~/ros2_ws/install/setup.bash
```

### 问题4：话题无响应

```bash
# 检查话题发布者
ros2 topic info /vision/target_pose

# 检查话题订阅者
ros2 topic info /task/command
```

---

## 下一步

测试完成后，可以：
1. 调整参数优化抓取精度
2. 添加更多测试场景
3. 集成真实硬件
4. 添加错误恢复机制