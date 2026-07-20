"""
robot_bringup.launch.py
-------------------------
Single entry point for everything that needs to run on the ground robot
itself. Wraps tb3_launcher's turtlebot3.launch.py (hardware + Nav2, an
external/lab-provided package we don't want to keep editing) and adds the
project's own robot-side nodes alongside it, so starting the robot only
takes one command instead of several separate terminals.
机器人端唯一的启动入口。把 tb3_launcher 的 turtlebot3.launch.py(硬件+Nav2,
外部/实验室提供的包,不想一直改它)包起来,同时把项目自己的机器人端节点
一起带上,这样启动机器人只需要一条命令,不用开好几个终端。

Included:
  - turtlebot3.launch.py (hardware bringup + Nav2)
  - map_receiver_node (receives a built map from the scheduler, no SSH)
  - clock_sync_node (lets the base station correct this robot's clock over
    a ROS2 service instead of SSH)

Not included on purpose:
  - initial_pose_publisher: one-shot and position-dependent, run manually
    after this launch is up and the robot is physically placed.

Usage / 用法:
    # No map argument: starts with no map loaded (map_server stays active,
    # just doesn't publish on /map until the scheduler injects one via
    # aerial recon). Use this to test the "no map yet" recon flow.
    # 不带map参数:不加载任何地图启动(map_server保持active,只是在调度器
    # 通过UAV建图注入一份之前,不会往/map发布任何东西)。用来测试"还没有
    # 地图"这条建图流程。
    ros2 launch ground_controller robot_bringup.launch.py

    # Explicit map: skips recon, scheduler reuses this map directly.
    # 显式指定地图:跳过建图,调度器直接复用这份地图。
    ros2 launch ground_controller robot_bringup.launch.py map:=<map yaml path>
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    tb3_launcher_dir = get_package_share_directory('tb3_launcher')

    # Empty by default: map_server configures/activates fine with no map
    # loaded and simply never publishes on /map until something calls
    # /map_server/load_map (confirmed empirically - it does NOT crash or
    # get stuck the way it does when pointed at a path that doesn't exist).
    # This is what lets scheduler_node's "does a map already exist" check
    # (current_map, populated from /map) correctly stay unset until the
    # scheduler actually injects one via aerial recon, instead of always
    # being pre-empted by a default map at startup.
    # Pass an explicit map:=<path> to skip recon and use that map directly.
    # 默认空字符串:map_server在没有地图的情况下能正常配置/激活,只是不会
    # 往/map发布任何东西,直到有人调用/map_server/load_map(实测验证过——
    # 跟指向一个不存在的路径不一样,不会崩溃或卡住)。这样才能让
    # scheduler_node判断"地图是否已存在"的依据(current_map,来自/map)
    # 正确保持未设置,直到调度器真正通过UAV建图注入一份地图,而不是一启动
    # 就被默认地图抢先满足。想跳过建图、直接用某份地图的话,显式传
    # map:=<路径>即可。
    map_arg = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to map yaml file to load (empty = start with no map)')

    turtlebot3_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_launcher_dir, 'launch', 'turtlebot3.launch.py')),
        launch_arguments={'map': LaunchConfiguration('map')}.items())

    map_receiver_node = Node(
        package='ground_controller',
        executable='map_receiver_node',
        name='map_receiver_node',
        output='screen',
    )

    clock_sync_node = Node(
        package='ground_controller',
        executable='clock_sync_node',
        name='clock_sync_node',
        output='screen',
    )

    return LaunchDescription([
        map_arg,
        turtlebot3_bringup,
        map_receiver_node,
        clock_sync_node,
    ])
