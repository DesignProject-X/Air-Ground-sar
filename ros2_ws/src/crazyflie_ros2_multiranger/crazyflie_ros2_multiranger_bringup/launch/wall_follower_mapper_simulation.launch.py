import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    # Configure ROS nodes for launch

    # Direction start_wall_following ends up hugging - see wall_following_multiranger's
    # own comments, the field is named for the search-turn direction so it's the
    # opposite of the wall it hugs (e.g. 'left' hugs the RIGHT wall).
    # 最终贴哪边墙的方向 - 命名是"搜索时的转向",跟实际贴的墙是反的
    # (比如 'left' 贴的是右墙),细节见 wall_following_multiranger 里的注释。
    takeoff_wallfollow_direction_arg = DeclareLaunchArgument(
        'wall_follow_direction',
        default_value='left',
    )

    # Setup project paths
    pkg_project_crazyflie_gazebo = get_package_share_directory('ros_gz_crazyflie_bringup')

    # Setup to launch a crazyflie gazebo simulation from the ros_gz_crazyflie project
    crazyflie_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_crazyflie_gazebo, 'launch', 'crazyflie_simulation.launch.py'))
    )

    # start a simple mapper node
    simple_mapper = Node(
        package='crazyflie_ros2_multiranger_simple_mapper',
        executable='simple_mapper_multiranger',
        name='simple_mapper',
        output='screen',
        parameters=[
            {'robot_prefix': 'crazyflie'},
            {'use_sim_time': True}
        ]
    )

    # start a wall following node with a delay of 5 seconds
    wall_following = Node(
        package='crazyflie_ros2_multiranger_wall_following',
        executable='wall_following_multiranger',
        name='wall_following',
        output='screen',
        parameters=[
            {'robot_prefix': 'crazyflie'},
            {'use_sim_time': True},
            {'delay': 5.0},
            # max_turn_rate matched to the real-hardware value (0.5, see
            # wall_follower_mapper_real.launch.py) so sim results transfer
            # more directly - sim used to run this at 0.7, faster than real
            # hardware's turning speed, which meant a smooth sim run didn't
            # necessarily predict real behavior.
            # max_forward_speed lowered to 0.15 per live observation while
            # watching it fly in Gazebo - 0.5 looked too fast in person, even
            # though it did complete a full loop and produced a good map.
            # max_turn_rate 对齐真机的值(0.5,见 wall_follower_mapper_real.
            # launch.py),这样仿真结果才能更直接地预测真机表现——之前仿真这里
            # 用的是0.7,比真机转得快,仿真跑得顺不代表真机也会顺。
            # max_forward_speed 降到 0.15,是根据实时观察 Gazebo 画面反馈调的——
            # 0.5 现场看着太快了,尽管它确实完整绕了一圈、建出了一张不错的图。
            {'max_turn_rate': 0.5},
            {'max_forward_speed': 0.15},
            {'wall_following_direction': 'right'}
        ]
    )

    rviz_config_path = os.path.join(
        get_package_share_directory('crazyflie_ros2_multiranger_bringup'),
        'config',
        'sim_mapping.rviz')

    rviz = Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            parameters=[{
                "use_sim_time": True
            }]
            )

    # Auto takeoff: climbs to hover height (watching real /odom feedback, not
    # a guessed delay) then calls wall_following's start_wall_following itself
    # and releases /cmd_vel - see cf_hover_sim.py's auto_wallfollow_direction
    # docs. A few seconds after everything else so control_services/the
    # start_wall_following service both actually exist by the time it starts
    # trying; it retries the service wait regardless, this just avoids log
    # spam from the first few failed attempts.
    # 自动起飞:爬升到悬停高度(靠真实 /odom 反馈判断,不是猜时间),然后自己去调
    # 巡墙节点的 start_wall_following,再让出 /cmd_vel - 细节见 cf_hover_sim.py
    # 里 auto_wallfollow_direction 的说明。比其他节点晚几秒起,只是为了避免它一
    # 上来 service 还没起来时刷一堆等待日志,不晚起也没事,反正它自己会重试等待。
    auto_takeoff = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='cf_controller',
                executable='cf_hover_sim',
                name='cf_hover_sim',
                output='screen',
                parameters=[
                    {'use_sim_time': True},
                    {'auto_wallfollow_direction': LaunchConfiguration('wall_follow_direction')},
                ]
            )
        ]
    )

    return LaunchDescription([
        takeoff_wallfollow_direction_arg,
        crazyflie_simulation,
        simple_mapper,
        wall_following,
        rviz,
        auto_takeoff,
        ])