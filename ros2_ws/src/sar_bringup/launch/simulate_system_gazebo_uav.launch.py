"""
simulate_system_gazebo_uav.launch.py
-------------------------------------
Full simulated mission pipeline, same as simulate_system.launch.py, but with
fake_agents/fake_uav_node.py (a static pre-baked map, no real flight) swapped
for the actual Gazebo Crazyflie stack (cf_controller/sim_uav_recon.launch.py):
a real simulated drone climbs, wall-follows the maze, and reports a MapResult
built from real flight data. fake_agents/fake_camera_node.py is dropped too,
since cf_mission_node_sim already fakes the target-detection signal itself
(tied to real flight progress, not a flat delay) - running both would double
-publish /camera/target_pose.

跟simulate_system.launch.py流程一样,只是把fake_agents/fake_uav_node.py
(静态预烤地图,没有真实飞行)换成真正的Gazebo Crazyflie那一套
(cf_controller/sim_uav_recon.launch.py):真实的仿真无人机会爬升、沿着迷宫
巡墙,用真实飞行数据建图后上报MapResult。同时也不再用
fake_agents/fake_camera_node.py,因为cf_mission_node_sim自己就会伪造目标
检测信号(跟着真实飞行进度走,不是一个死延时)——两个一起跑会导致
/camera/target_pose被发布两次。

Usage / 用法:
    ros2 launch sar_bringup simulate_system_gazebo_uav.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('sar_bringup'),
        'config', 'params.yaml'
    )

    pkg_project_crazyflie_gazebo = get_package_share_directory('ros_gz_crazyflie_bringup')
    crazyflie_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_crazyflie_gazebo, 'launch', 'crazyflie_simulation.launch.py')))

    simple_mapper = Node(
        package='crazyflie_ros2_multiranger_simple_mapper',
        executable='simple_mapper_multiranger',
        name='simple_mapper',
        output='screen',
        parameters=[
            {'robot_prefix': 'crazyflie'},
            {'use_sim_time': True},
        ],
    )

    wall_following = Node(
        package='crazyflie_ros2_multiranger_wall_following',
        executable='wall_following_multiranger',
        name='wall_following',
        output='screen',
        parameters=[
            {'robot_prefix': 'crazyflie'},
            {'use_sim_time': True},
            {'max_turn_rate': 0.5},
            {'max_forward_speed': 0.15},
        ],
    )

    rviz_config_path = os.path.join(
        get_package_share_directory('crazyflie_ros2_multiranger_bringup'),
        'config', 'sim_mapping.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}],
    )

    # No auto_wallfollow_direction param - dispatch-driven, waits for the
    # scheduler's /uav/dispatch instead of climbing on a launch-time timer.
    # 不设置auto_wallfollow_direction参数——派发驱动,等调度器的
    # /uav/dispatch,而不是启动后按固定延时自动爬升。
    cf_hover_sim = Node(
        package='cf_controller',
        executable='cf_hover_sim',
        name='cf_hover_sim',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # Uses cf_mission_node_sim.py's own mapping_duration_sec default (120s)
    # - scheduler_node.RECON_TIMEOUT_S is 140s specifically to leave margin
    # above that, so no override needed here.
    # 用cf_mission_node_sim.py自己的mapping_duration_sec默认值(120秒)——
    # scheduler_node.RECON_TIMEOUT_S设成140秒就是专门为了在这之上留余量,
    # 这里不需要再单独覆盖。
    cf_mission_node_sim = Node(
        package='cf_controller',
        executable='cf_mission_node_sim',
        name='cf_mission_node_sim',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        crazyflie_simulation,
        simple_mapper,
        wall_following,
        rviz,
        cf_hover_sim,
        cf_mission_node_sim,
        Node(
            package='scheduling',
            executable='scheduler_node',
            parameters=[params],
        ),
        Node(
            package='fake_agents',
            executable='fake_ground_node',
        ),
        Node(
            package='ai_planning',
            executable='planner_node',
        ),
    ])
