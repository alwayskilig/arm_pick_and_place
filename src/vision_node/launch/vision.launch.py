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
