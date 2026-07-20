"""
initial_pose.launch.py
------------------------
Publishes a fixed initial pose to /initialpose so AMCL is seeded without
needing to click "2D Pose Estimate" in RViz every time - place the ground
robot at (approximately) the same physical spot each test run, then run
this instead of the manual RViz step.
自动发布一个固定的初始位姿到 /initialpose,不用每次都在RViz里手动点
"2D Pose Estimate" - 每次测试把小车摆在(大致)同一个物理位置,跑这个
代替手动点RViz那一步。

Defaults below were read directly from a live /amcl_pose while the robot
was sitting at the test's usual starting spot - re-run and update these if
that spot changes.
下面这组默认值是小车摆在测试常用起点时,直接读的一次 /amcl_pose - 如果
起点变了,要重新读一次、更新这里的默认值。

Usage / 用法:
    ros2 launch ground_controller initial_pose.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    x_arg = DeclareLaunchArgument('x', default_value='-0.405')
    y_arg = DeclareLaunchArgument('y', default_value='-0.230')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0')

    initial_pose_publisher = Node(
        package='ground_controller',
        executable='initial_pose_publisher',
        name='initial_pose_publisher',
        output='screen',
        parameters=[{
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'yaw': LaunchConfiguration('yaw'),
        }],
    )

    return LaunchDescription([
        x_arg,
        y_arg,
        yaw_arg,
        initial_pose_publisher,
    ])
