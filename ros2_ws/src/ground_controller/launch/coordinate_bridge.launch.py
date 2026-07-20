"""
coordinate_bridge.launch.py
----------------------------
Starts coordinate_bridge_node with the ArUco marker's measured mounting
offset baked in as defaults, so it doesn't depend on remembering to pass
these on the command line every time (that's exactly how the yaw got left
at 0 before and silently broke target-position accuracy for a while).
以带默认值的方式启动 coordinate_bridge_node,把ArUco标记实测的安装偏移量
固定下来,不用每次手动带命令行参数(之前yaw就是因为一直没传、默认成了0,
悄悄导致目标定位不准了一段时间)。

These defaults come from physically measuring where the marker sits
relative to base_link, plus the yaw found by placing a target directly in
front of the robot and checking which axis of /camera/target_pose_raw
picked up that "forward" distance (see conversation history / commit
message for that test). If the marker ever gets remounted, re-measure and
update the defaults below - or override on the command line without
touching this file:
    ros2 launch ground_controller coordinate_bridge.launch.py marker_offset_yaw:=<new value>
这些默认值来自实测marker相对base_link的安装位置,以及把目标放在小车正
前方、看/camera/target_pose_raw里哪个轴接住了这个"正前方"距离,反推出来的
yaw(细节见对话记录/这次提交的说明)。如果marker以后重新安装了,需要重新
测量、更新下面的默认值——或者不改这个文件,直接在命令行覆盖:
    ros2 launch ground_controller coordinate_bridge.launch.py marker_offset_yaw:=<新值>

Usage / 用法:
    ros2 launch ground_controller coordinate_bridge.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    marker_offset_x_arg = DeclareLaunchArgument('marker_offset_x', default_value='-0.12')
    marker_offset_y_arg = DeclareLaunchArgument('marker_offset_y', default_value='0.04')
    marker_offset_z_arg = DeclareLaunchArgument('marker_offset_z', default_value='0.0')
    marker_offset_roll_arg = DeclareLaunchArgument('marker_offset_roll', default_value='0.0')
    marker_offset_pitch_arg = DeclareLaunchArgument('marker_offset_pitch', default_value='0.0')
    # -pi/2: found empirically - a target placed directly in front of the
    # robot showed up on the marker's Y axis instead of X, meaning the
    # marker frame is rotated -90 deg (yaw) relative to base_link.
    # -pi/2:实测得出——把目标放在小车正前方,结果落在了marker的Y轴而不是
    # X轴上,说明marker坐标系相对base_link转了-90度(yaw)。
    marker_offset_yaw_arg = DeclareLaunchArgument('marker_offset_yaw', default_value='-1.5708')
    # amcl's own map->odom broadcast rate is slower and less regular than
    # the ~2Hz target detection rate - especially while the robot is sitting
    # still - so the default 0.5s wait was routinely too short and caused
    # intermittent "extrapolation into the future" transform failures.
    # amcl自己发布map->odom的频率比目标检测的~2Hz要慢、也不够规律——尤其
    # 小车静止不动的时候——默认0.5秒的等待经常不够,导致间歇性出现
    # "extrapolation into the future"的转换失败。
    transform_timeout_arg = DeclareLaunchArgument('transform_timeout_sec', default_value='2.0')

    coordinate_bridge_node = Node(
        package='ground_controller',
        executable='coordinate_bridge_node',
        name='coordinate_bridge_node',
        output='screen',
        parameters=[{
            'marker_offset_x': LaunchConfiguration('marker_offset_x'),
            'marker_offset_y': LaunchConfiguration('marker_offset_y'),
            'marker_offset_z': LaunchConfiguration('marker_offset_z'),
            'marker_offset_roll': LaunchConfiguration('marker_offset_roll'),
            'marker_offset_pitch': LaunchConfiguration('marker_offset_pitch'),
            'marker_offset_yaw': LaunchConfiguration('marker_offset_yaw'),
            'transform_timeout_sec': LaunchConfiguration('transform_timeout_sec'),
        }],
    )

    return LaunchDescription([
        marker_offset_x_arg,
        marker_offset_y_arg,
        marker_offset_z_arg,
        marker_offset_roll_arg,
        marker_offset_pitch_arg,
        marker_offset_yaw_arg,
        transform_timeout_arg,
        coordinate_bridge_node,
    ])
