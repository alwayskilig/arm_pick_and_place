from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 获取各包的share目录
    wpr_sim_dir = get_package_share_directory('wpr_simulation2')
    
    # 获取参数文件路径
    config_dir = os.path.join(
        get_package_share_directory('arm_pick_and_place'),
        'config'
    )
    param_file = os.path.join(config_dir, 'params.yaml')
    
    return LaunchDescription([
        # 启动wpr_simulation2仿真环境（WPR1机械臂场景）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(wpr_sim_dir, 'launch', 'wpr1_simple.launch.py')
            )
        ),
        
        # 启动云台控制节点（用于扩展视觉视野）
        Node(
            package='pan_tilt_controller',
            executable='pan_tilt_controller',
            name='pan_tilt_controller',
            parameters=[param_file],
            output='screen'
        ),
        
        # 启动视觉节点
        Node(
            package='vision_node',
            executable='vision_node',
            name='vision_node',
            parameters=[param_file],
            output='screen'
        ),
        
        # 启动机械臂控制节点
        Node(
            package='arm_controller',
            executable='arm_controller',
            name='arm_controller',
            parameters=[param_file],
            output='screen'
        ),
        
        # 启动夹爪控制节点
        Node(
            package='gripper_controller',
            executable='gripper_controller',
            name='gripper_controller',
            parameters=[param_file],
            output='screen'
        ),
        
        # 启动任务调度节点
        Node(
            package='task_scheduler',
            executable='task_scheduler',
            name='task_scheduler',
            parameters=[param_file],
            output='screen'
        ),
    ])
