"""
base_station_real.launch.py
-----------------------------
Single entry point for everything that needs to run on the base station for
a real-hardware mission (camera perception + coordinate transform + mission
control). Pairs with ground_controller's robot_bringup.launch.py, which
covers the other machine.
基站端唯一的启动入口,面向真实硬件任务(相机感知 + 坐标转换 + 任务控制)。
和机器人端的 robot_bringup.launch.py 配对使用,那边负责另一台机器。

Included:
  - RealSense camera driver (depth aligned to color)
  - target_detector_node (YOLO + ArUco detection, continuous - not single_shot)
  - coordinate_bridge_node (via ground_controller's own launch file, so the
    measured marker offset/yaw defaults stay in one place)
  - ground_controller_node (forwards goals to Nav2)
  - scheduler_node (mission state machine)
  - clock_sync_client (one-shot: pushes this machine's time to the ground
    robot's clock_sync_node over a ROS2 service - fire-and-forget, doesn't
    block or depend on the other nodes above)
  - rosbridge_websocket (WebSocket gateway for the dashboard.html web UI)
  - planner_node (natural-language operator command -> TaskCommand, via LLM)

Not included on purpose:
  - initial_pose_publisher: one-shot and position-dependent, run manually
    once the robot is physically placed and both ends are up.

Usage / 用法:
    ros2 launch sar_bringup base_station_real.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    realsense_share_dir = get_package_share_directory('realsense2_camera')
    ground_controller_share_dir = get_package_share_directory('ground_controller')
    rosbridge_share_dir = get_package_share_directory('rosbridge_server')

    realsense_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_share_dir, 'launch', 'rs_launch.py')),
        launch_arguments={'align_depth.enable': 'true'}.items())

    rosbridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(rosbridge_share_dir, 'launch', 'rosbridge_websocket_launch.xml')))

    planner_node = Node(
        package='ai_planning',
        executable='planner_node',
        name='planner_node',
        output='screen',
    )

    target_detector_node = Node(
        package='uav_vision',
        executable='target_detector_node',
        name='target_detector_node',
        output='screen',
        parameters=[{
            'save_tile_debug_dir': '/home/sutd/01_SAP/Air-Ground-sar/ros2_ws/src/uav_vision/debug_output',
        }],
    )

    coordinate_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ground_controller_share_dir, 'launch', 'coordinate_bridge.launch.py')))

    ground_controller_node = Node(
        package='ground_controller',
        executable='ground_controller_node',
        name='ground_controller_node',
        output='screen',
    )

    scheduler_node = Node(
        package='scheduling',
        executable='scheduler_node',
        name='scheduler_node',
        output='screen',
    )

    clock_sync_client = Node(
        package='ground_controller',
        executable='clock_sync_client',
        name='clock_sync_client',
        output='screen',
    )

    return LaunchDescription([
        realsense_camera,
        target_detector_node,
        coordinate_bridge,
        ground_controller_node,
        scheduler_node,
        clock_sync_client,
        rosbridge,
        planner_node,
    ])
