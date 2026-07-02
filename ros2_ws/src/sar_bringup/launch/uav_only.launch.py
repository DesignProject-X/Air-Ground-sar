from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('sar_bringup'),
        'config', 'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='scheduling',
            executable='scheduler_node',
            parameters=[params],
        ),
        Node(
            package='fake_agents',
            executable='fake_uav_node',
        ),
    ])
