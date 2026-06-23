from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('arm_pick_and_place'),
        'config', 'params.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='vision_node',
            executable='vision_node',
            name='vision_node',
            output='screen',
            parameters=[config_path]
        ),
        Node(
            package='arm_controller',
            executable='arm_controller',
            name='arm_controller',
            output='screen',
            parameters=[config_path]
        ),
        Node(
            package='gripper_controller',
            executable='gripper_controller',
            name='gripper_controller',
            output='screen',
            parameters=[config_path]
        ),
        Node(
            package='task_scheduler',
            executable='task_scheduler',
            name='task_scheduler',
            output='screen',
            parameters=[config_path]
        ),
    ])