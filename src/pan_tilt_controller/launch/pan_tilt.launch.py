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
            package='pan_tilt_controller',
            executable='pan_tilt_controller',
            name='pan_tilt_controller',
            parameters=[param_file],
            output='screen'
        ),
    ])
