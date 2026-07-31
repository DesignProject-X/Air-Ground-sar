import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, SetRemap

# Known /cmd_vel collision (see docs/testing/real_hardware.md): wall_following
# and vel_mux both hardcode the bare, unnamespaced '/cmd_vel', assuming the
# drone is the only robot on the network. On real hardware this launch runs
# alongside a real ground robot under the same ROS_DOMAIN_ID, and a real
# TurtleBot3 base controller also listens on that same bare '/cmd_vel' -
# reproduced live: the instant the drone started wall-following, the ground
# robot started moving too, in sync, with the scheduler still stuck in RECON
# and no map ever sent - the drone's raw hover/wall-following Twist was
# reaching the real robot's motors directly, nothing to do with navigation or
# mission state at all. Already fixed for the simulated drone in
# sim_uav_recon.launch.py; this applies the exact same remap here for the
# real one.
# 已知的/cmd_vel冲突问题(见docs/testing/real_hardware.md):wall_following和
# vel_mux都硬编码用了没加命名空间的裸'/cmd_vel',假设无人机是网络上唯一的
# 机器人。真机测试时这个launch是跟真实地面机器人在同一个ROS_DOMAIN_ID下一起
# 跑的,真实TurtleBot3的底盘控制器也监听同一个裸'/cmd_vel'——实测复现过:
# 无人机一开始沿墙飞,地面机器人也同步跟着动了,当时调度器还卡在RECON、
# 地图压根没发过去——无人机原始的悬停/巡墙Twist直接被真实机器人的电机收走
# 了,跟导航、任务状态毫无关系。仿真无人机那边在sim_uav_recon.launch.py里
# 已经修过这个问题了,这里给真机也套用完全一样的remap。
REAL_CMD_VEL_TOPIC = '/crazyflie/hover_cmd_vel'


def generate_launch_description():
    # Configure ROS nodes for launch
    # Applies for the rest of this LaunchDescription, including nodes started
    # by the crazyflie launch.py include below - see the module docstring
    # comment above for why this is needed.
    # 对这个LaunchDescription剩下的部分都生效,包括下面include的crazyflie
    # launch.py里启动的节点——原因见上面模块开头的注释。
    cmd_vel_remap = SetRemap(src='/cmd_vel', dst=REAL_CMD_VEL_TOPIC)

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
    # assume_airborne=True: cf_mission_node already takes off and flies to
    # the start position before ever calling start_wall_following, so
    # vel_mux's own "first /cmd_vel -> takeoff" auto-takeoff would otherwise
    # fire redundantly right as wall-following starts, sending a second
    # takeoff to a drone already at hover height (observed to cause a
    # dip-and-reclimb glitch at the start of every real wall-following run).
    # assume_airborne=True:cf_mission_node在调用start_wall_following之前,
    # 就已经让无人机起飞并飞到了起点,所以vel_mux自己"收到第一条/cmd_vel就
    # 自动起飞"的逻辑,在这里每次都会对着一台已经在悬停高度的无人机多发一次
    # 多余的起飞指令(实测会在每次真机沿墙飞行刚开始时造成一次"原地下降再
    # 爬升"的诡异动作)。
    crazyflie_vel_mux = Node(
            package='crazyflie_examples',
            executable='vel_mux',
            name='vel_mux',
            output='screen',
            parameters=[{'hover_height': 0.2},
                        {'incoming_twist_topic': '/cmd_vel'},
                        {'robot_prefix': 'crazyflie_real'},
                        {'assume_airborne': True},],
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
        cmd_vel_remap,
        crazyflie_real,
        crazyflie_vel_mux,
        map_world_bridge,
        simple_mapper,
        wall_following,
        cf_mission,
        rviz
        ])