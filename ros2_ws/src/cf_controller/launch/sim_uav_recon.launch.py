"""
sim_uav_recon.launch.py
-------------------------
Dispatch-driven variant of crazyflie_ros2_multiranger_bringup's
wall_follower_mapper_simulation.launch.py: brings up the same Gazebo
Crazyflie + simple_mapper + wall_following stack, but the drone only
starts flying in response to the scheduler's /uav/dispatch instead of
auto-taking-off on a fixed timer the moment this launch comes up.
crazyflie_ros2_multiranger_bringup 的 wall_follower_mapper_simulation.
launch.py 的派发驱动版本:起同一套 Gazebo Crazyflie + 建图 + 巡墙,但无人机
只有收到调度器的 /uav/dispatch 才会开始飞,而不是这个 launch 一起来就在固定
延时后自动起飞。

Adds cf_mission_node_sim, which fakes a target detection after a fixed
mapping duration (neither drone carries a camera - see that node's
docstring for why) and reports the resulting map as MapResult on
/uav/map_result, replacing fake_agents/fake_uav_node.py for a more
realistic (if still not real-hardware) recon test.
加了 cf_mission_node_sim,固定建图时长后伪造一次目标检测(无人机不管真机还是
仿真都不带摄像头,原因见该节点的说明),把建好的地图打包成 MapResult 发到
/uav/map_result,用来替代 fake_agents/fake_uav_node.py,做一次更贴近真实
(虽然还不是真实硬件)的建图测试。

cf_hover_sim.py and wall_following_multiranger.py both hardcode their
publisher as the bare, unnamespaced '/cmd_vel' (by design of the upstream
crazyflie_ros2_multiranger project, which assumes the sim is the only
robot on the network) - control_services.py (started via the included
crazyflie_simulation.launch.py) subscribes to that same bare '/cmd_vel' and
relays it to the namespaced /crazyflie/cmd_vel that actually drives the
simulated drone via the Gazebo bridge. That's fine when the sim runs alone,
but this launch is meant to run alongside a REAL ground robot on the same
ROS_DOMAIN_ID (see docs/testing/real_hardware.md) - a real TurtleBot3 base
controller also listens on that same bare '/cmd_vel' for its own driving
commands. Reproduced live: as soon as the sim drone started flying, the
real ground robot started moving too, with the scheduler still in RECON
and no map ever sent - the sim drone's raw hover/wall-following Twist was
reaching the real robot's motors directly, nothing to do with navigation or
mission state at all. SetRemap below redirects every reference to the bare
'/cmd_vel' (cf_hover_sim's and wall_following's publishers, plus
control_services' subscriber inside the included launch file) onto a name
that can't collide - control_services' own /crazyflie/cmd_vel output (a
different, already-namespaced string) is untouched, so the Gazebo bridge
still works.
cf_hover_sim.py 和 wall_following_multiranger.py 都把自己的发布话题写死成了
没加命名空间的'/cmd_vel'(上游crazyflie_ros2_multiranger项目的设计,假设仿真是
网络上唯一的机器人)——control_services.py(通过下面include的
crazyflie_simulation.launch.py启动)订阅同一个没加命名空间的'/cmd_vel',再转发
到真正通过Gazebo桥接驱动仿真无人机的、加了命名空间的/crazyflie/cmd_vel。仿真
单独跑没问题,但这个launch是要跟真实地面机器人在同一个ROS_DOMAIN_ID下一起跑的
(见docs/testing/real_hardware.md)——真实TurtleBot3的底盘控制器也监听同一个
没加命名空间的'/cmd_vel'来接收驱动指令。实测复现过:仿真无人机一开始飞,真实
地面机器人也跟着动了,当时调度器还在RECON、地图压根没发下去——仿真无人机原始
的悬停/巡墙Twist直接被真实机器人的电机收走了,跟导航、任务状态毫无关系。下面
的SetRemap把所有引用没加命名空间'/cmd_vel'的地方(cf_hover_sim和
wall_following的发布者,以及include进来的launch文件里control_services的
订阅者)都重定向到一个不会冲突的话题名——control_services自己的
/crazyflie/cmd_vel输出(是另一个、已经加了命名空间的字符串)不受影响,Gazebo
桥接照常工作。

Usage / 用法:
    ros2 launch cf_controller sim_uav_recon.launch.py
    # Paired with a real ground camera (target_detector_node) instead of
    # fake_agents - stops cf_mission_node_sim from also publishing a fake
    # /camera/target_pose, which would race the real camera's detection:
    # 跟真实地面相机(target_detector_node)搭配,而不是fake_agents——阻止
    # cf_mission_node_sim再发一个假的/camera/target_pose,跟真实相机的检测抢跑:
    ros2 launch cf_controller sim_uav_recon.launch.py fake_target_signal:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue

SIM_CMD_VEL_TOPIC = '/crazyflie/hover_cmd_vel'


def generate_launch_description():
    fake_target_signal_arg = DeclareLaunchArgument(
        'fake_target_signal', default_value='true',
        description=(
            'Whether cf_mission_node_sim fakes a /camera/target_pose after '
            'mapping. Set to false when a real ground camera '
            '(target_detector_node) is also running, so target detection is '
            'left entirely to it instead of racing a fake signal from here.'))

    # Applies for the rest of this LaunchDescription, including nodes
    # started by the crazyflie_simulation.launch.py include below - see the
    # module docstring for why this is needed.
    # 对这个LaunchDescription剩下的部分都生效,包括下面include的
    # crazyflie_simulation.launch.py里启动的节点——原因见文件开头的说明。
    cmd_vel_remap = SetRemap(src='/cmd_vel', dst=SIM_CMD_VEL_TOPIC)

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
            {'delay': 5.0},
            {'max_turn_rate': 0.5},
            {'max_forward_speed': 0.15},
            {'wall_following_direction': 'right'},
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

    # No auto_wallfollow_direction param here (unlike
    # wall_follower_mapper_simulation.launch.py's auto_takeoff) - leaving it
    # unset puts cf_hover_sim in dispatch-driven mode, waiting for
    # /uav/dispatch instead of climbing on a fixed launch-time timer.
    # 这里不给 auto_wallfollow_direction 参数(跟
    # wall_follower_mapper_simulation.launch.py 的 auto_takeoff 不一样)——
    # 不设置这个参数会让 cf_hover_sim 进入派发驱动模式,等 /uav/dispatch
    # 而不是在固定的启动延时后自动爬升。
    cf_hover_sim = Node(
        package='cf_controller',
        executable='cf_hover_sim',
        name='cf_hover_sim',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    cf_mission_node_sim = Node(
        package='cf_controller',
        executable='cf_mission_node_sim',
        name='cf_mission_node_sim',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            # A raw LaunchConfiguration resolves to a plain string
            # ('true'/'false'), which would clash with cf_mission_node_sim's
            # bool-typed declare_parameter default - ParameterValue with
            # value_type=bool does the string->bool cast launch_ros expects.
            # 直接用LaunchConfiguration解析出来是纯字符串('true'/'false'),
            # 会跟cf_mission_node_sim里bool类型的declare_parameter默认值冲突
            # ——用ParameterValue指定value_type=bool做launch_ros要求的
            # 字符串到布尔值的转换。
            {'fake_target_signal': ParameterValue(
                LaunchConfiguration('fake_target_signal'), value_type=bool)},
        ],
    )

    return LaunchDescription([
        fake_target_signal_arg,
        cmd_vel_remap,
        crazyflie_simulation,
        simple_mapper,
        wall_following,
        rviz,
        cf_hover_sim,
        cf_mission_node_sim,
    ])
