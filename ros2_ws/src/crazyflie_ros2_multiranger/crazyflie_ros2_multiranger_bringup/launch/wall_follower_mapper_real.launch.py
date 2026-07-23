import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    # Configure ROS nodes for launch
    # TEMP DIAGNOSTIC: vel_mux.py connects to the same Crazyflie over its own
    # independent crazyflie_py/Crazyswarm() radio link, entirely separate
    # from crazyflie_server's own cflib connection - i.e. two simultaneous,
    # independent connections to one physical drone over the same dongle at
    # once. Suspected cause of a real-hardware takeoff going sideways
    # (isolated takeoff service call - not through wall_following/vel_mux's
    # own Twist path at all - still drifted, and the exact same takeoff
    # mechanism worked fine standalone through cfclient once vel_mux/
    # crazyflie_server weren't both connected). This flag lets vel_mux be
    # left out to test crazyflie_server on its own, isolating whether the
    # dual connection is the actual cause before deciding on a permanent fix.
    # 临时诊断用:vel_mux.py是用它自己独立的crazyflie_py/Crazyswarm()射频连接
    # 接到同一台Crazyflie上的,跟crazyflie_server自己的cflib连接完全是两条
    # 独立的连接——也就是说,同一个加密狗、同一台无人机,同时被两个独立连接
    # 占用。怀疑这是真机测试里"单独调起飞服务(完全没走wall_following/
    # vel_mux那条Twist路径)结果飞偏了,而同样的起飞机制单独在cfclient里
    # (vel_mux/crazyflie_server都没连着)却工作正常"这个现象的原因。这个
    # 开关能让vel_mux先不起,单独测crazyflie_server,在决定怎么永久修复之前,
    # 先确认是不是真的是这个双重连接导致的。
    enable_vel_mux_arg = DeclareLaunchArgument(
        'enable_vel_mux', default_value='true',
        description=(
            'Whether to start vel_mux (needed for wall-following flight, '
            'which streams Twist to /cmd_vel). Set to false to isolate '
            'crazyflie_server on its own - e.g. to test whether vel_mux\'s '
            'own independent radio connection to the same Crazyflie is '
            'interfering with plain Takeoff/GoTo service calls.'))

    # Setup project paths'''
    pkg_project_crazyswarm2 = get_package_share_directory('crazyflie')
    pkg_multiranger_bringup = get_package_share_directory('crazyflie_ros2_multiranger_bringup')
    crazyflies_yaml = os.path.join(
        pkg_multiranger_bringup,
        'config',
        'crazyflie_real_crazyswarm2.yaml')

    # Start up a crazyflie server through the Crazyswarm2 project
    crazyflie_real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_project_crazyswarm2, 'launch'), '/launch.py']),
        launch_arguments={'crazyflies_yaml_file': crazyflies_yaml, 'backend': 'cflib', 'mocap': 'False', 'rviz': 'False'}.items()
    )

    # Start a velocity multiplexer node for the crazyflie
    crazyflie_vel_mux = Node(
            package='crazyflie_examples',
            executable='vel_mux',
            name='vel_mux',
            output='screen',
            parameters=[{'hover_height': 0.2},
                        {'incoming_twist_topic': '/cmd_vel'},
                        {'robot_prefix': 'crazyflie_real'},],
            condition=IfCondition(LaunchConfiguration('enable_vel_mux')),
        )

    # Bridge the mapper's static 'map' tree to the driver's live 'world' tree
    # (crazyflie_server broadcasts a live world->crazyflie_real transform,
    # but simple_mapper only ever links map->crazyflie_real/odom - without
    # this, they're two disconnected TF trees and rviz can't show the live
    # drone frame while Fixed Frame is 'map'.)
    map_world_bridge = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_world_bridge',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'world']
    )

    # start a simple mapper node
    simple_mapper = Node(
        package='crazyflie_ros2_multiranger_simple_mapper',
        executable='simple_mapper_multiranger',
        name='simple_mapper',
        output='screen',
        parameters=[
            {'robot_prefix': 'crazyflie_real'},
            {'use_sim_time': False}
        ]
    )

    # start a wall following node; it waits idle for a start_wall_following
    # service call (issued by cf_mission_node) instead of auto-starting
    wall_following = Node(
        package='crazyflie_ros2_multiranger_wall_following',
        executable='wall_following_multiranger',
        name='wall_following',
        output='screen',
        parameters=[
            {'robot_prefix': 'crazyflie_real'},
            {'use_sim_time': False},
            {'max_turn_rate': 0.5},
            {'max_forward_speed': 0.15},
        ]
    )

    # bridge scheduler dispatch (start point + direction) and camera target
    # detection to real flight control
    cf_mission = Node(
        package='cf_controller',
        executable='cf_mission_node',
        name='cf_mission_node',
        output='screen',
        parameters=[
            {'robot_prefix': 'crazyflie_real'},
        ]
    )

    rviz_config_path = os.path.join(
        get_package_share_directory('crazyflie_ros2_multiranger_bringup'),
        'config',
        'real_mapping.rviz')

    rviz = Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            parameters=[{
                "use_sim_time": False
            }]
            )

    return LaunchDescription([
        enable_vel_mux_arg,
        crazyflie_real,
        crazyflie_vel_mux,
        map_world_bridge,
        simple_mapper,
        wall_following,
        cf_mission,
        rviz
        ])