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
