"""
control_simulation.launch.py
-----------------------------
控制测试专用 launch 文件。
只启动：
  - Gazebo 仿真环境（含 ROS↔Gazebo 桥接、control_services）
  - RViz 可视化
不启动：
  - wall_following（沿墙飞行）
  - simple_mapper（建图）
这样启动后，/cmd_vel 话题只有你自己的控制节点在发，不会被抢控制权。
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    # 复用 ros_gz_crazyflie 的仿真 launch（含 Gazebo + 桥接 + control_services）
    pkg_project_crazyflie_gazebo = get_package_share_directory('ros_gz_crazyflie_bringup')
    crazyflie_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_crazyflie_gazebo, 'launch', 'crazyflie_simulation.launch.py'))
    )

    # RViz（复用原来的配置文件，保持显示一致）
    rviz_config_path = os.path.join(
        get_package_share_directory('crazyflie_ros2_multiranger_bringup'),
        'config',
        'sim_mapping.rviz'
    )
    rviz = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}]
    )

    # 只启动这两个，不加 wall_following 和 simple_mapper
    return LaunchDescription([
        crazyflie_simulation,
        rviz,
    ])