"""
real_robot_launch.py
---------------------
实机航点飞行专用 launch 文件。
启动内容：
  - crazyswarm2 驱动（cflib backend，直连真实无人机）
  - RViz 可视化
不启动：
  - Gazebo（实机不需要）
  - wall_following / simple_mapper

用法：
    ros2 launch cf_controller real_robot_launch.py

注意：
  - 启动前确认 Crazyradio 已插上、无人机已开机
  - 无人机放在平坦地面上、保持静止等待起飞
  - 话题前缀是 /cf231（不是仿真里的 /crazyflie）
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # 复用 crazyswarm2 的主 launch，指定用 cflib backend 直连真机
    crazyflie_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('crazyflie'),
                'launch',
                'launch.py'
            )
        ),
        launch_arguments={
            'backend': 'cflib',       # 用 cflib 直连真实无人机
            'mocap': 'False',         # 不用动捕系统
            'teleop': 'False',        # 不用手柄遥控
            'rviz': 'True',           # 启动 RViz
        }.items()
    )

    return LaunchDescription([
        crazyflie_launch,
    ])