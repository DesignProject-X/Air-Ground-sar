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
    # Without this, scheduler_node's _load_zones() only declares its
    # zone_names default (['default'], direction='right') - 'zone_a'/'zone_b'
    # are never registered as known zones at all, so a goal_zone of 'zone_b'
    # silently falls back to 'default' with the wrong direction, instead of
    # erroring loudly. Reproduced live: real-hardware full-pipeline test
    # dispatched with goal_zone='zone_b' expecting direction='left', but the
    # drone turned the opposite way at its first wall - this params file is
    # what actually defines zone_a/zone_b (see sar_bringup/config/params.yaml).
    # 不加这个的话,scheduler_node的_load_zones()只会声明zone_names的默认值
    # (['default'],direction='right')——'zone_a'/'zone_b'根本没被注册成
    # 已知的zone,所以goal_zone传'zone_b'会悄悄地退回到'default'、用错误的
    # 方向,而不是报错。真机全流程测试实测复现过:派发时用goal_zone='zone_b'、
    # 以为对应direction='left',结果无人机在第一面墙就转错了方向——真正定义
    # zone_a/zone_b的正是这个参数文件(见sar_bringup/config/params.yaml)。
    scheduler_params = os.path.join(
        get_package_share_directory('sar_bringup'), 'config', 'params.yaml')

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
        parameters=[scheduler_params],
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
