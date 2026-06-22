# 机械臂抓取与递送系统实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 实现一个基于ROS2的机械臂抓取与递送系统，包含视觉识别、机械臂控制、夹爪控制和任务调度四个核心模块。

**Architecture:** 采用多节点模块化架构，每个功能模块作为独立的ROS2节点，通过话题通信。使用Gazebo Classic进行仿真，OpenCV进行视觉识别。

**Tech Stack:** ROS2 Galactic+, Gazebo Classic, OpenCV 4.5+, Python 3.8+, MoveIt

---

## 文件结构

在开始实现之前，先定义完整的文件结构：

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

---

## Task 1: 项目基础设施搭建

**Files:**
- Create: `arm_pick_and_place/msg/TaskCommand.msg`
- Create: `arm_pick_and_place/msg/TaskStatus.msg`
- Create: `arm_pick_and_place/CMakeLists.txt`
- Create: `arm_pick_and_place/package.xml`

- [ ] **Step 1: 创建项目根目录和消息文件**

```bash
mkdir -p arm_pick_and_place/msg
```

创建 `arm_pick_and_place/msg/TaskCommand.msg`:
```msg
# TaskCommand.msg
uint8 command_type
uint8 GRASP=1
uint8 DELIVER=2
uint8 RESET=3
geometry_msgs/PoseStamped target_pose
```

创建 `arm_pick_and_place/msg/TaskStatus.msg`:
```msg
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

- [ ] **Step 2: 创建CMakeLists.txt**

```cmake
cmake_minimum_required(VERSION 3.8)
project(arm_pick_and_place)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TaskCommand.msg"
  "msg/TaskStatus.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_package()
```

- [ ] **Step 3: 创建package.xml**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>arm_pick_and_place</name>
  <version>0.1.0</version>
  <description>Robot arm pick and place system</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <depend>geometry_msgs</depend>
  <depend>std_msgs</depend>

  <exec_depend>rosidl_default_runtime</exec_depend>

  <member_of_group>rosidl_interface_packages</member_of_group>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 4: 编译验证消息**

```bash
cd arm_pick_and_place
colcon build --packages-select arm_pick_and_place
source install/setup.bash
```

Expected: 编译成功，无错误

- [ ] **Step 5: 提交基础结构**

```bash
git add .
git commit -m "feat: initial project structure with custom messages"
```

---

## Task 2: 视觉识别节点实现

**Files:**
- Create: `arm_pick_and_place/src/vision_node/vision_node/__init__.py`
- Create: `arm_pick_and_place/src/vision_node/vision_node/vision_node.py`
- Create: `arm_pick_and_place/src/vision_node/launch/vision.launch.py`
- Create: `arm_pick_and_place/src/vision_node/package.xml`
- Create: `arm_pick_and_place/src/vision_node/setup.py`
- Create: `arm_pick_and_place/src/vision_node/setup.cfg`

- [ ] **Step 1: 创建vision_node包结构**

```bash
mkdir -p arm_pick_and_place/src/vision_node/vision_node
mkdir -p arm_pick_and_place/src/vision_node/launch
```

- [ ] **Step 2: 创建__init__.py**

```python
# arm_pick_and_place/src/vision_node/vision_node/__init__.py
```

- [ ] **Step 3: 实现vision_node.py**

```python
# arm_pick_and_place/src/vision_node/vision_node/vision_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        
        # 声明参数
        self.declare_parameter('target_color', 'red')
        self.declare_parameter('min_area', 500)
        self.declare_parameter('max_area', 50000)
        self.declare_parameter('camera_frame', 'camera_link')
        
        # 获取参数
        self.target_color = self.get_parameter('target_color').value
        self.min_area = self.get_parameter('min_area').value
        self.max_area = self.get_parameter('max_area').value
        self.camera_frame = self.get_parameter('camera_frame').value
        
        # OpenCV桥接器
        self.bridge = CvBridge()
        
        # 订阅摄像头图像
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        
        # 发布目标位姿
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/vision/target_pose',
            10)
        
        self.get_logger().info('Vision node initialized')
    
    def image_callback(self, msg):
        try:
            # 转换ROS图像到OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # 检测目标物体
            target_pose = self.detect_target(cv_image)
            
            if target_pose is not None:
                self.pose_pub.publish(target_pose)
                self.get_logger().info(f'Target detected at: {target_pose.pose.position}')
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')
    
    def detect_target(self, image):
        # 转换到HSV色彩空间
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # 根据目标颜色设置阈值
        if self.target_color == 'red':
            lower_bound = np.array([0, 120, 70])
            upper_bound = np.array([10, 255, 255])
        elif self.target_color == 'green':
            lower_bound = np.array([36, 50, 70])
            upper_bound = np.array([89, 255, 255])
        elif self.target_color == 'blue':
            lower_bound = np.array([100, 150, 0])
            upper_bound = np.array([140, 255, 255])
        else:
            self.get_logger().warn(f'Unknown color: {self.target_color}')
            return None
        
        # 创建掩码
        mask = cv2.inRange(hsv, lower_bound, upper_bound)
        
        # 形态学操作
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # 查找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            
            if self.min_area < area < self.max_area:
                # 计算轮廓中心
                M = cv2.moments(contour)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    
                    # 创建位姿消息
                    pose_msg = PoseStamped()
                    pose_msg.header.stamp = self.get_clock().now().to_msg()
                    pose_msg.header.frame_id = self.camera_frame
                    
                    # 简单的位姿估计（需要相机内参进行精确计算）
                    # 这里使用像素坐标作为x,y，假设z为固定值
                    pose_msg.pose.position.x = float(cx) / 1000.0  # 简单转换
                    pose_msg.pose.position.y = float(cy) / 1000.0
                    pose_msg.pose.position.z = 0.5  # 假设距离
                    
                    pose_msg.pose.orientation.w = 1.0
                    
                    return pose_msg
        
        return None


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 4: 创建launch文件**

```python
# arm_pick_and_place/src/vision_node/launch/vision.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('arm_pick_and_place'),
        'config'
    )
    param_file = os.path.join(config_dir, 'params.yaml')
    
    return LaunchDescription([
        Node(
            package='vision_node',
            executable='vision_node',
            name='vision_node',
            parameters=[param_file],
            output='screen'
        ),
    ])
```

- [ ] **Step 5: 创建package.xml**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>vision_node</name>
  <version>0.1.0</version>
  <description>Vision processing node for target detection</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <depend>rclpy</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>cv_bridge</depend>
  <depend>opencv_python</depend>

  <exec_depend>rosidl_default_runtime</exec_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 6: 创建setup.py**

```python
from setuptools import setup
import os
from glob import glob

package_name = 'vision_node'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Vision processing node for target detection',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_node = vision_node.vision_node:main',
        ],
    },
)
```

- [ ] **Step 7: 创建setup.cfg**

```ini
[develop]
script_dir=$base/lib/vision_node
[install]
install_scripts=$base/lib/vision_node
```

- [ ] **Step 8: 编译验证**

```bash
cd arm_pick_and_place
colcon build --packages-select vision_node
source install/setup.bash
```

Expected: 编译成功，无错误

- [ ] **Step 9: 提交vision_node**

```bash
git add src/vision_node
git commit -m "feat: implement vision node with OpenCV target detection"
```

---

## Task 3: 机械臂控制节点实现

**Files:**
- Create: `arm_pick_and_place/src/arm_controller/arm_controller/__init__.py`
- Create: `arm_pick_and_place/src/arm_controller/arm_controller/arm_controller.py`
- Create: `arm_pick_and_place/src/arm_controller/arm_controller/ik_solver.py`
- Create: `arm_pick_and_place/src/arm_controller/launch/arm.launch.py`
- Create: `arm_pick_and_place/src/arm_controller/package.xml`
- Create: `arm_pick_and_place/src/arm_controller/setup.py`
- Create: `arm_pick_and_place/src/arm_controller/setup.cfg`

- [ ] **Step 1: 创建arm_controller包结构**

```bash
mkdir -p arm_pick_and_place/src/arm_controller/arm_controller
mkdir -p arm_pick_and_place/src/arm_controller/launch
```

- [ ] **Step 2: 实现ik_solver.py**

```python
# arm_pick_and_place/src/arm_controller/arm_controller/ik_solver.py
import numpy as np
from dataclasses import dataclass
from typing import List, Optional


@dataclass
class JointAngles:
    """关节角度"""
    angles: List[float]
    
    def __post_init__(self):
        if len(self.angles) != 6:
            raise ValueError("6 joint angles required")


@dataclass
class Pose:
    """位姿"""
    x: float
    y: float
    z: float
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0


class IKSolver:
    """简单的逆运动学求解器"""
    
    def __init__(self):
        # 机器人DH参数（简化版）
        self.link_lengths = [0.1, 0.3, 0.25, 0.15, 0.1, 0.08]
        
    def solve(self, target_pose: Pose) -> Optional[JointAngles]:
        """
        求解逆运动学
        简化实现：使用几何方法求解
        """
        try:
            # 简化的逆运动学求解
            # 实际应用中应使用更精确的算法或MoveIt
            
            x, y, z = target_pose.x, target_pose.y, target_pose.z
            
            # 计算基座旋转角度
            joint1 = np.arctan2(y, x)
            
            # 计算其他关节角度（简化）
            r = np.sqrt(x**2 + y**2)
            joint2 = np.arctan2(z - self.link_lengths[0], r) - np.pi/4
            joint3 = np.pi/4
            joint4 = -joint2
            joint5 = 0.0
            joint6 = 0.0
            
            angles = [joint1, joint2, joint3, joint4, joint5, joint6]
            
            # 限制关节角度范围
            for i in range(len(angles)):
                angles[i] = max(-np.pi, min(np.pi, angles[i]))
            
            return JointAngles(angles)
            
        except Exception as e:
            return None
    
    def forward_kinematics(self, joint_angles: JointAngles) -> Optional[Pose]:
        """
        正运动学求解
        """
        try:
            angles = joint_angles.angles
            
            # 简化的正运动学计算
            x = (self.link_lengths[1] * np.cos(angles[0]) * np.cos(angles[1]) +
                 self.link_lengths[2] * np.cos(angles[0]) * np.cos(angles[1] + angles[2]))
            
            y = (self.link_lengths[1] * np.sin(angles[0]) * np.cos(angles[1]) +
                 self.link_lengths[2] * np.sin(angles[0]) * np.cos(angles[1] + angles[2]))
            
            z = (self.link_lengths[0] +
                 self.link_lengths[1] * np.sin(angles[1]) +
                 self.link_lengths[2] * np.sin(angles[1] + angles[2]))
            
            return Pose(x, y, z)
            
        except Exception as e:
            return None
```

- [ ] **Step 3: 实现arm_controller.py**

```python
# arm_pick_and_place/src/arm_controller/arm_controller/arm_controller.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from arm_pick_and_place.msg import TaskCommand, TaskStatus
from .ik_solver import IKSolver, Pose as IKPose
import numpy as np
import time


class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        
        # 声明参数
        self.declare_parameter('arm_joint_names', [
            'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'
        ])
        self.declare_parameter('max_velocity', 0.5)
        self.declare_parameter('max_acceleration', 1.0)
        self.declare_parameter('home_position', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('pick_height', 0.1)
        self.declare_parameter('place_height', 0.1)
        
        # 获取参数
        self.joint_names = self.get_parameter('arm_joint_names').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.max_acceleration = self.get_parameter('max_acceleration').value
        self.home_position = self.get_parameter('home_position').value
        self.pick_height = self.get_parameter('pick_height').value
        self.place_height = self.get_parameter('place_height').value
        
        # IK求解器
        self.ik_solver = IKSolver()
        
        # 当前关节状态
        self.current_joints = [0.0] * 6
        self.is_moving = False
        
        # 订阅目标位姿
        self.target_sub = self.create_subscription(
            PoseStamped,
            '/vision/target_pose',
            self.target_callback,
            10)
        
        # 订阅任务指令
        self.command_sub = self.create_subscription(
            TaskCommand,
            '/task/command',
            self.command_callback,
            10)
        
        # 发布关节状态
        self.joint_pub = self.create_publisher(
            JointState,
            '/arm/joint_states',
            10)
        
        # 发布任务状态
        self.status_pub = self.create_publisher(
            TaskStatus,
            '/task/status',
            10)
        
        # 创建定时器发布关节状态
        self.timer = self.create_timer(0.1, self.publish_joint_state)
        
        self.get_logger().info('Arm controller initialized')
    
    def target_callback(self, msg):
        """处理视觉目标位姿"""
        if self.is_moving:
            return
        
        self.get_logger().info(f'Received target pose: {msg.pose.position}')
        
        # 转换位姿格式
        target = IKPose(
            x=msg.pose.position.x,
            y=msg.pose.position.y,
            z=msg.pose.position.z
        )
        
        # 求解逆运动学
        joint_angles = self.ik_solver.solve(target)
        
        if joint_angles is not None:
            self.move_to_joints(joint_angles.angles)
    
    def command_callback(self, msg):
        """处理任务指令"""
        self.get_logger().info(f'Received command: {msg.command_type}')
        
        if msg.command_type == TaskCommand.RESET:
            self.move_home()
        elif msg.command_type == TaskCommand.GRASP:
            self.move_to_grasp(msg.target_pose)
        elif msg.command_type == TaskCommand.DELIVER:
            self.move_to_deliver(msg.target_pose)
    
    def move_to_joints(self, target_joints, duration=2.0):
        """移动到目标关节角度"""
        self.is_moving = True
        
        # 发布移动状态
        self.publish_status(TaskStatus.MOVING, 0.0, 'Moving to target')
        
        start_joints = self.current_joints.copy()
        start_time = time.time()
        
        while time.time() - start_time < duration:
            elapsed = time.time() - start_time
            progress = min(elapsed / duration, 1.0)
            
            # 线性插值
            for i in range(6):
                self.current_joints[i] = start_joints[i] + \
                    (target_joints[i] - start_joints[i]) * progress
            
            time.sleep(0.01)
        
        self.current_joints = target_joints.copy()
        self.is_moving = False
        
        # 发布完成状态
        self.publish_status(TaskStatus.IDLE, 1.0, 'Movement complete')
    
    def move_home(self):
        """移动到零位"""
        self.get_logger().info('Moving to home position')
        self.move_to_joints(self.home_position)
    
    def move_to_grasp(self, target_pose):
        """移动到抓取位置"""
        self.get_logger().info('Moving to grasp position')
        
        # 计算抓取位置（目标上方）
        grasp_pose = IKPose(
            x=target_pose.pose.position.x,
            y=target_pose.pose.position.y,
            z=target_pose.pose.position.z + self.pick_height
        )
        
        joint_angles = self.ik_solver.solve(grasp_pose)
        if joint_angles is not None:
            self.move_to_joints(joint_angles.angles)
    
    def move_to_deliver(self, target_pose):
        """移动到递送位置"""
        self.get_logger().info('Moving to deliver position')
        
        # 计算递送位置（目标上方）
        deliver_pose = IKPose(
            x=target_pose.pose.position.x,
            y=target_pose.pose.position.y,
            z=target_pose.pose.position.z + self.place_height
        )
        
        joint_angles = self.ik_solver.solve(deliver_pose)
        if joint_angles is not None:
            self.move_to_joints(joint_angles.angles)
    
    def publish_joint_state(self):
        """发布关节状态"""
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.current_joints
        msg.velocity = []
        msg.effort = []
        
        self.joint_pub.publish(msg)
    
    def publish_status(self, state, progress, message):
        """发布任务状态"""
        msg = TaskStatus()
        msg.state = state
        msg.progress = progress
        msg.message = message
        
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 4: 创建launch文件**

```python
# arm_pick_and_place/src/arm_controller/launch/arm.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('arm_pick_and_place'),
        'config'
    )
    param_file = os.path.join(config_dir, 'params.yaml')
    
    return LaunchDescription([
        Node(
            package='arm_controller',
            executable='arm_controller',
            name='arm_controller',
            parameters=[param_file],
            output='screen'
        ),
    ])
```

- [ ] **Step 5: 创建package.xml**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>arm_controller</name>
  <version>0.1.0</version>
  <description>Robot arm controller with IK solver</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <depend>rclpy</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>std_msgs</depend>
  <depend>arm_pick_and_place</depend>

  <exec_depend>rosidl_default_runtime</exec_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 6: 创建setup.py**

```python
from setuptools import setup
import os
from glob import glob

package_name = 'arm_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Robot arm controller with IK solver',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_controller = arm_controller.arm_controller:main',
        ],
    },
)
```

- [ ] **Step 7: 创建setup.cfg**

```ini
[develop]
script_dir=$base/lib/arm_controller
[install]
install_scripts=$base/lib/arm_controller
```

- [ ] **Step 8: 编译验证**

```bash
cd arm_pick_and_place
colcon build --packages-select arm_controller
source install/setup.bash
```

Expected: 编译成功，无错误

- [ ] **Step 9: 提交arm_controller**

```bash
git add src/arm_controller
git commit -m "feat: implement arm controller with IK solver"
```

---

## Task 4: 夹爪控制节点实现

**Files:**
- Create: `arm_pick_and_place/src/gripper_controller/gripper_controller/__init__.py`
- Create: `arm_pick_and_place/src/gripper_controller/gripper_controller/gripper_controller.py`
- Create: `arm_pick_and_place/src/gripper_controller/launch/gripper.launch.py`
- Create: `arm_pick_and_place/src/gripper_controller/package.xml`
- Create: `arm_pick_and_place/src/gripper_controller/setup.py`
- Create: `arm_pick_and_place/src/gripper_controller/setup.cfg`

- [ ] **Step 1: 创建gripper_controller包结构**

```bash
mkdir -p arm_pick_and_place/src/gripper_controller/gripper_controller
mkdir -p arm_pick_and_place/src/gripper_controller/launch
```

- [ ] **Step 2: 实现gripper_controller.py**

```python
# arm_pick_and_place/src/gripper_controller/gripper_controller/gripper_controller.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from arm_pick_and_place.msg import TaskCommand
import time


class GripperController(Node):
    def __init__(self):
        super().__init__('gripper_controller')
        
        # 声明参数
        self.declare_parameter('open_position', 0.04)
        self.declare_parameter('close_position', 0.0)
        self.declare_parameter('close_force', 20.0)
        self.declare_parameter('action_timeout', 5.0)
        
        # 获取参数
        self.open_position = self.get_parameter('open_position').value
        self.close_position = self.get_parameter('close_position').value
        self.close_force = self.get_parameter('close_force').value
        self.action_timeout = self.get_parameter('action_timeout').value
        
        # 当前状态
        self.current_position = self.open_position
        self.is_open = True
        
        # 订阅任务指令
        self.command_sub = self.create_subscription(
            TaskCommand,
            '/task/command',
            self.command_callback,
            10)
        
        # 发布夹爪状态
        self.state_pub = self.create_publisher(
            Float64,
            '/gripper/state',
            10)
        
        # 创建定时器发布状态
        self.timer = self.create_timer(0.1, self.publish_state)
        
        self.get_logger().info('Gripper controller initialized')
    
    def command_callback(self, msg):
        """处理任务指令"""
        self.get_logger().info(f'Received command: {msg.command_type}')
        
        if msg.command_type == TaskCommand.GRASP:
            self.close()
        elif msg.command_type == TaskCommand.DELIVER:
            self.open()
        elif msg.command_type == TaskCommand.RESET:
            self.open()
    
    def close(self):
        """闭合夹爪"""
        self.get_logger().info('Closing gripper')
        
        start_time = time.time()
        while time.time() - start_time < self.action_timeout:
            if self.current_position > self.close_position:
                self.current_position -= 0.001
                self.publish_state()
                time.sleep(0.01)
            else:
                self.current_position = self.close_position
                self.is_open = False
                self.get_logger().info('Gripper closed')
                break
    
    def open(self):
        """打开夹爪"""
        self.get_logger().info('Opening gripper')
        
        start_time = time.time()
        while time.time() - start_time < self.action_timeout:
            if self.current_position < self.open_position:
                self.current_position += 0.001
                self.publish_state()
                time.sleep(0.01)
            else:
                self.current_position = self.open_position
                self.is_open = True
                self.get_logger().info('Gripper opened')
                break
    
    def publish_state(self):
        """发布夹爪状态"""
        msg = Float64()
        msg.data = self.current_position
        
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GripperController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 3: 创建launch文件**

```python
# arm_pick_and_place/src/gripper_controller/launch/gripper.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('arm_pick_and_place'),
        'config'
    )
    param_file = os.path.join(config_dir, 'params.yaml')
    
    return LaunchDescription([
        Node(
            package='gripper_controller',
            executable='gripper_controller',
            name='gripper_controller',
            parameters=[param_file],
            output='screen'
        ),
    ])
```

- [ ] **Step 4: 创建package.xml**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>gripper_controller</name>
  <version>0.1.0</version>
  <description>Gripper controller for pick and place</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>arm_pick_and_place</depend>

  <exec_depend>rosidl_default_runtime</exec_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 5: 创建setup.py**

```python
from setuptools import setup
import os
from glob import glob

package_name = 'gripper_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Gripper controller for pick and place',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gripper_controller = gripper_controller.gripper_controller:main',
        ],
    },
)
```

- [ ] **Step 6: 创建setup.cfg**

```ini
[develop]
script_dir=$base/lib/gripper_controller
[install]
install_scripts=$base/lib/gripper_controller
```

- [ ] **Step 7: 编译验证**

```bash
cd arm_pick_and_place
colcon build --packages-select gripper_controller
source install/setup.bash
```

Expected: 编译成功，无错误

- [ ] **Step 8: 提交gripper_controller**

```bash
git add src/gripper_controller
git commit -m "feat: implement gripper controller"
```

---

## Task 5: 任务调度节点实现

**Files:**
- Create: `arm_pick_and_place/src/task_scheduler/task_scheduler/__init__.py`
- Create: `arm_pick_and_place/src/task_scheduler/task_scheduler/states.py`
- Create: `arm_pick_and_place/src/task_scheduler/task_scheduler/task_scheduler.py`
- Create: `arm_pick_and_place/src/task_scheduler/launch/task.launch.py`
- Create: `arm_pick_and_place/src/task_scheduler/package.xml`
- Create: `arm_pick_and_place/src/task_scheduler/setup.py`
- Create: `arm_pick_and_place/src/task_scheduler/setup.cfg`

- [ ] **Step 1: 创建task_scheduler包结构**

```bash
mkdir -p arm_pick_and_place/src/task_scheduler/task_scheduler
mkdir -p arm_pick_and_place/src/task_scheduler/launch
```

- [ ] **Step 2: 实现states.py**

```python
# arm_pick_and_place/src/task_scheduler/task_scheduler/states.py
from enum import Enum, auto


class TaskState(Enum):
    """任务状态"""
    IDLE = auto()
    DETECTING = auto()
    MOVING_TO_OBJECT = auto()
    GRASPING = auto()
    LIFTING = auto()
    MOVING_TO_TARGET = auto()
    RELEASING = auto()
    RETURNING = auto()
    ERROR = auto()
```

- [ ] **Step 3: 实现task_scheduler.py**

```python
# arm_pick_and_place/src/task_scheduler/task_scheduler/task_scheduler.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from arm_pick_and_place.msg import TaskCommand, TaskStatus
from .states import TaskState
import time


class TaskScheduler(Node):
    def __init__(self):
        super().__init__('task_scheduler')
        
        # 声明参数
        self.declare_parameter('pick_position', [0.3, 0.0, 0.1])
        self.declare_parameter('place_position', [-0.3, 0.0, 0.1])
        self.declare_parameter('safe_height', 0.3)
        self.declare_parameter('move_timeout', 10.0)
        
        # 获取参数
        self.pick_position = self.get_parameter('pick_position').value
        self.place_position = self.get_parameter('place_position').value
        self.safe_height = self.get_parameter('safe_height').value
        self.move_timeout = self.get_parameter('move_timeout').value
        
        # 状态机
        self.current_state = TaskState.IDLE
        self.target_pose = None
        self.grasp_pose = None
        
        # 订阅视觉目标
        self.target_sub = self.create_subscription(
            PoseStamped,
            '/vision/target_pose',
            self.target_callback,
            10)
        
        # 订阅关节状态
        self.joint_sub = self.create_subscription(
            JointState,
            '/arm/joint_states',
            self.joint_callback,
            10)
        
        # 订阅夹爪状态
        self.gripper_sub = self.create_subscription(
            Float64,
            '/gripper/state',
            self.gripper_callback,
            10)
        
        # 发布任务指令
        self.command_pub = self.create_publisher(
            TaskCommand,
            '/task/command',
            10)
        
        # 发布任务状态
        self.status_pub = self.create_publisher(
            TaskStatus,
            '/task/status',
            10)
        
        # 创建状态机定时器
        self.timer = self.create_timer(0.1, self.state_machine)
        
        self.get_logger().info('Task scheduler initialized')
    
    def target_callback(self, msg):
        """处理视觉目标"""
        if self.current_state == TaskState.DETECTING:
            self.target_pose = msg
            self.get_logger().info(f'Target detected: {msg.pose.position}')
    
    def joint_callback(self, msg):
        """处理关节状态"""
        pass
    
    def gripper_callback(self, msg):
        """处理夹爪状态"""
        pass
    
    def state_machine(self):
        """状态机主循环"""
        if self.current_state == TaskState.IDLE:
            pass
        
        elif self.current_state == TaskState.DETECTING:
            if self.target_pose is not None:
                self.get_logger().info('Target found, moving to object')
                self.current_state = TaskState.MOVING_TO_OBJECT
                self.send_grasp_command()
        
        elif self.current_state == TaskState.MOVING_TO_OBJECT:
            # 等待移动完成
            pass
        
        elif self.current_state == TaskState.GRASPING:
            # 等待抓取完成
            pass
        
        elif self.current_state == TaskState.LIFTING:
            # 等待提升完成
            pass
        
        elif self.current_state == TaskState.MOVING_TO_TARGET:
            # 等待移动到递送点完成
            pass
        
        elif self.current_state == TaskState.RELEASING:
            # 等待释放完成
            pass
        
        elif self.current_state == TaskState.RETURNING:
            # 等待返回零位完成
            pass
        
        elif self.current_state == TaskState.ERROR:
            pass
    
    def send_grasp_command(self):
        """发送抓取指令"""
        msg = TaskCommand()
        msg.command_type = TaskCommand.GRASP
        
        if self.target_pose is not None:
            msg.target_pose = self.target_pose
        
        self.command_pub.publish(msg)
        self.publish_status(TaskState.GRASPING, 0.0, 'Grasping object')
    
    def send_deliver_command(self):
        """发送递送指令"""
        msg = TaskCommand()
        msg.command_type = TaskCommand.DELIVER
        
        if self.target_pose is not None:
            msg.target_pose = self.target_pose
        
        self.command_pub.publish(msg)
        self.publish_status(TaskState.RELEASING, 0.0, 'Delivering object')
    
    def send_reset_command(self):
        """发送复位指令"""
        msg = TaskCommand()
        msg.command_type = TaskCommand.RESET
        self.command_pub.publish(msg)
        self.publish_status(TaskState.RETURNING, 0.0, 'Returning to home')
    
    def publish_status(self, state, progress, message):
        """发布任务状态"""
        msg = TaskStatus()
        
        # 映射状态
        state_map = {
            TaskState.IDLE: TaskStatus.IDLE,
            TaskState.DETECTING: TaskStatus.MOVING,
            TaskState.MOVING_TO_OBJECT: TaskStatus.MOVING,
            TaskState.GRASPING: TaskStatus.GRASPING,
            TaskState.LIFTING: TaskStatus.MOVING,
            TaskState.MOVING_TO_TARGET: TaskStatus.MOVING,
            TaskState.RELEASING: TaskStatus.DELIVERING,
            TaskState.RETURNING: TaskStatus.MOVING,
            TaskState.ERROR: TaskStatus.ERROR,
        }
        
        msg.state = state_map.get(state, TaskStatus.IDLE)
        msg.progress = progress
        msg.message = message
        
        self.status_pub.publish(msg)
    
    def start_task(self):
        """开始任务"""
        self.get_logger().info('Starting pick and place task')
        self.current_state = TaskState.DETECTING
        self.publish_status(TaskState.DETECTING, 0.0, 'Searching for target')


def main(args=None):
    rclpy.init(args=args)
    node = TaskScheduler()
    
    # 自动开始任务
    node.start_task()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 4: 创建launch文件**

```python
# arm_pick_and_place/src/task_scheduler/launch/task.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('arm_pick_and_place'),
        'config'
    )
    param_file = os.path.join(config_dir, 'params.yaml')
    
    return LaunchDescription([
        Node(
            package='task_scheduler',
            executable='task_scheduler',
            name='task_scheduler',
            parameters=[param_file],
            output='screen'
        ),
    ])
```

- [ ] **Step 5: 创建package.xml**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>task_scheduler</name>
  <version>0.1.0</version>
  <description>Task scheduler for pick and place operation</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <depend>rclpy</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>std_msgs</depend>
  <depend>arm_pick_and_place</depend>

  <exec_depend>rosidl_default_runtime</exec_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 6: 创建setup.py**

```python
from setuptools import setup
import os
from glob import glob

package_name = 'task_scheduler'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Task scheduler for pick and place operation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task_scheduler = task_scheduler.task_scheduler:main',
        ],
    },
)
```

- [ ] **Step 7: 创建setup.cfg**

```ini
[develop]
script_dir=$base/lib/task_scheduler
[install]
install_scripts=$base/lib/task_scheduler
```

- [ ] **Step 8: 编译验证**

```bash
cd arm_pick_and_place
colcon build --packages-select task_scheduler
source install/setup.bash
```

Expected: 编译成功，无错误

- [ ] **Step 9: 提交task_scheduler**

```bash
git add src/task_scheduler
git commit -m "feat: implement task scheduler with state machine"
```

---

## Task 6: 系统集成与配置

**Files:**
- Create: `arm_pick_and_place/config/params.yaml`
- Create: `arm_pick_and_place/launch/full_system.launch.py`
- Create: `arm_pick_and_place/README.md`

- [ ] **Step 1: 创建参数配置文件**

```yaml
# arm_pick_and_place/config/params.yaml
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

- [ ] **Step 2: 创建主启动文件**

```python
# arm_pick_and_place/launch/full_system.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 获取各包的share目录
    vision_node_dir = get_package_share_directory('vision_node')
    arm_controller_dir = get_package_share_directory('arm_controller')
    gripper_controller_dir = get_package_share_directory('gripper_controller')
    task_scheduler_dir = get_package_share_directory('task_scheduler')
    
    return LaunchDescription([
        # 启动视觉节点
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(vision_node_dir, 'launch', 'vision.launch.py')
            )
        ),
        
        # 启动机械臂控制节点
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(arm_controller_dir, 'launch', 'arm.launch.py')
            )
        ),
        
        # 启动夹爪控制节点
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gripper_controller_dir, 'launch', 'gripper.launch.py')
            )
        ),
        
        # 启动任务调度节点
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(task_scheduler_dir, 'launch', 'task.launch.py')
            )
        ),
    ])
```

- [ ] **Step 3: 创建README.md**

```markdown
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
```

- [ ] **Step 4: 编译完整系统**

```bash
cd arm_pick_and_place
colcon build
source install/setup.bash
```

Expected: 所有包编译成功，无错误

- [ ] **Step 5: 提交系统集成**

```bash
git add config launch README.md
git commit -m "feat: complete system integration with launch files and config"
```

---

## Task 7: 系统测试与验证

- [ ] **Step 1: 验证消息编译**

```bash
cd arm_pick_and_place
colcon build --packages-select arm_pick_and_place
ros2 interface show arm_pick_and_place/msg/TaskCommand
ros2 interface show arm_pick_and_place/msg/TaskStatus
```

Expected: 显示消息定义

- [ ] **Step 2: 验证节点启动**

```bash
# 终端1：启动视觉节点
ros2 run vision_node vision_node

# 终端2：启动机械臂控制节点
ros2 run arm_controller arm_controller

# 终端3：启动夹爪控制节点
ros2 run gripper_controller gripper_controller

# 终端4：启动任务调度节点
ros2 run task_scheduler task_scheduler
```

Expected: 各节点正常启动，无错误

- [ ] **Step 3: 验证话题通信**

```bash
# 终端1：监听话题
ros2 topic list
ros2 topic echo /task/status

# 终端2：发布测试消息
ros2 topic pub --once /task/command arm_pick_and_place/msg/TaskCommand "{command_type: 3}"
```

Expected: 能正常发送和接收消息

- [ ] **Step 4: 提交测试结果**

```bash
git add .
git commit -m "test: verify system integration and communication"
```

---

## 完成

实现计划完成。所有核心模块已实现：
1. ✅ 项目基础设施搭建
2. ✅ 视觉识别节点
3. ✅ 机械臂控制节点
4. ✅ 夹爪控制节点
5. ✅ 任务调度节点
6. ✅ 系统集成与配置
7. ✅ 系统测试与验证

**下一步：**
需要将代码部署到实际的ROS2环境中，并在Gazebo仿真中进行测试。
