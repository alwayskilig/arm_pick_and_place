# PadBot E2 硬件集成指南

## 概述
本指南说明如何将PadBot E2硬件集成到机械臂抓取系统中。

## 需要修改的文件

### 1. `padbot_robot_arm` 包中的 `dobot.cpp`
**位置**：`padbot_E2/padbot_robot_arm/padbot_robot_arm/src/dobot.cpp`

**修改内容**：
1. 添加目标位姿订阅者
2. 实现目标位姿到Dobot PTP命令的转换

**修改步骤**：
1. 在文件开头添加头文件：
   ```cpp
   #include "geometry_msgs/msg/pose_stamped.hpp"
   ```

2. 在类定义中添加订阅者成员：
   ```cpp
   rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub;
   ```

3. 在构造函数中创建订阅者：
   ```cpp
   target_pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
       "/arm/target_pose", 10, std::bind(&DobotController::target_pose_callback, this, std::placeholders::_1)
   );
   ```

4. 添加回调函数：
   ```cpp
   void target_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
       if (openDobotState != DobotConnect_NoError) {
           return;
       }
       
       // 将米转换为毫米
       double x = msg->pose.position.x * 1000.0;
       double y = msg->pose.position.y * 1000.0;
       double z = msg->pose.position.z * 1000.0;
       
       // 设置PTP命令
       PTPCmd cmd;
       cmd.ptpMode = 0;
       cmd.x = x;
       cmd.y = y;
       cmd.z = z;
       cmd.r = 0;
       
       // 发送命令
       uint64_t queuedCmdIndex;
       SetPTPCmd(&cmd, true, &queuedCmdIndex);
       RCLCPP_INFO(rclcpp::get_logger("dobot_controller"), "Moving to target: x=%.2f, y=%.2f, z=%.2f", x, y, z);
   }
   ```

### 2. `padbot_pan_tilt` 包
**位置**：`padbot_E2/padbot_pan_tilt/padbot_pan_tilt/`

**消息定义**：
- `PanTiltControl.msg`：云台控制指令
- `PanTiltInfo.msg`：云台状态信息

**使用方法**：
1. 订阅`/pan_tilt/state`话题获取云台状态
2. 发布`PanTiltControl`消息到`/pan_tilt_control`话题控制云台

### 3. 修改主项目代码

#### 修改云台控制节点
**文件**：`src/pan_tilt_controller/pan_tilt_controller/pan_tilt_controller.py`

**修改内容**：
1. 导入`padbot_pan_tilt`消息
2. 修改发布者为`PanTiltControl`消息
3. 修改话题为`/pan_tilt_control`

#### 修改任务调度节点
**文件**：`src/task_scheduler/task_scheduler/task_scheduler.py`

**修改内容**：
1. 添加`/arm/target_pose`发布者
2. 在发送抓取/递送命令时同时发布目标位姿

## 编译步骤

### 1. 编译 `padbot_robot_arm`
```bash
cd ~/ros2_ws
colcon build --packages-select padbot_robot_arm
source install/setup.bash
```

### 2. 编译 `padbot_pan_tilt`
```bash
colcon build --packages-select padbot_pan_tilt
source install/setup.bash
```

### 3. 编译主项目
```bash
colcon build --packages-select arm_pick_and_place
source install/setup.bash
```

## 创建硬件启动文件

创建 `launch/full_system_hardware.launch.py`：
```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    vision_node_dir = get_package_share_directory('vision_node')
    arm_controller_dir = get_package_share_directory('arm_controller')
    gripper_controller_dir = get_package_share_directory('gripper_controller')
    task_scheduler_dir = get_package_share_directory('task_scheduler')
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(vision_node_dir, 'launch', 'vision.launch.py'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(arm_controller_dir, 'launch', 'arm.launch.py'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gripper_controller_dir, 'launch', 'gripper.launch.py'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(task_scheduler_dir, 'launch', 'task.launch.py'))
        ),
        Node(
            package='padbot_robot_arm',
            executable='dobot_controller',
            name='dobot_controller',
            output='screen'
        ),
        Node(
            package='padbot_pan_tilt',
            executable='pan_tilt_node',
            name='pan_tilt_node',
            output='screen'
        ),
    ])
```

## 测试硬件连接

### 1. 测试Dobot连接
```bash
# 检查串口权限
ls -l /dev/dobot

# 测试Dobot控制器
ros2 run padbot_robot_arm dobot_controller
```

### 2. 测试云台连接
```bash
# 测试云台控制器
ros2 run padbot_pan_tilt pan_tilt_node

# 发送测试指令
ros2 topic pub /pan_tilt_control padbot_pan_tilt/msg/PanTiltControl "{angular_vel_y: 0.0, angle_y: 10.0, angular_vel_z: 0.0, angle_z: 0.0}"
```

### 3. 测试完整系统
```bash
# 启动完整系统
ros2 launch arm_pick_and_place full_system_hardware.launch.py

# 监控话题
ros2 topic list
ros2 topic echo /arm/target_pose
```

## 故障排除

### 1. Dobot连接失败
- 检查串口权限：`sudo chmod 666 /dev/dobot`
- 检查Dobot是否上电
- 检查串口波特率：115200

### 2. 云台无响应
- 检查云台电源
- 检查ROS2话题是否正常：`ros2 topic list | grep pan_tilt`

### 3. 视觉识别不准确
- 校准摄像头内参
- 调整颜色检测阈值
- 确保光照条件良好

## 参数配置

在 `config/params.yaml` 中添加硬件参数：
```yaml
vision_node:
  ros__parameters:
    camera_topic: "/camera/image_raw"  # 实际摄像头话题
    target_color: "red"
    
arm_controller:
  ros__parameters:
    use_hardware: true  # 使用Dobot硬件
```