#!/usr/bin/env python3
"""
Velocity multiplexer example using crazyflie_py.

Subscribes to /cmd_vel (Twist) and handles takeoff, hover, and landing
automatically, eliminating the need for the separate vel_mux ROS node.

Velocity mapping from the incoming Twist message:
  linear.x  -> vx (m/s forward)
  linear.y  -> vy (m/s left)
  angular.z -> yaw_rate (rad/s)
  linear.z  -> < 0 triggers landing; >= 0 continues hovering

Usage:
  ros2 run crazyflie_examples vel_mux --ros-args -p hover_height:=0.5
Then send commands with, e.g.:
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""

from crazyflie_py import Crazyswarm
from geometry_msgs.msg import Twist
import rclpy


HOVER_HEIGHT = 0.3
# 0.3m/1.0s = 0.3 m/s climb/descent rate - matches the rate confirmed safe via
# manual Takeoff service testing in this project's small physical maze, where
# a slower climb (0.3m/3.0s = 0.1 m/s) spent too long low to the ground and
# was destabilized by propwash/ground-effect turbulence off the nearby walls.
TAKEOFF_DURATION = 1.0
LAND_HEIGHT = 0.05


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]

    # When another node (e.g. cf_mission_node) already took the drone off
    # and flew it to a start position before ever starting to publish
    # /cmd_vel, this vel_mux's own "first cmd_vel -> takeoff" logic below is
    # redundant - it fires anyway as soon as it sees the first nonzero
    # command, sending a second takeoff to a drone already at hover_height,
    # which was observed to cause a dip-and-reclimb glitch right as
    # wall-following started. Set this true from launch in that case; the
    # standalone teleop_twist_keyboard usage in this file's docstring (and
    # simple_mapper_real.launch.py) has nothing else doing the takeoff, so it
    # needs this to stay false and keep auto-taking-off on first command.
    # 如果另一个节点(比如cf_mission_node)在还没开始发/cmd_vel之前,就已经
    # 让无人机起飞并飞到了起点,那下面这段vel_mux自己"收到第一条cmd_vel就
    # 起飞"的逻辑就是多余的——它还是会在看到第一条非零指令时触发,对一台
    # 已经在悬停高度的无人机又发一次起飞,实测会在沿墙飞行刚开始时造成一次
    # "原地下降再爬升"的诡异动作。这种情况下从launch里把这个参数设成true。
    # 这个文件docstring里说的那种独立配合teleop_twist_keyboard手动遥控的用法
    # (以及simple_mapper_real.launch.py)没有别的节点会去起飞,所以要保持
    # false,让它继续在收到第一条指令时自动起飞。
    swarm.allcfs.declare_parameter('assume_airborne', False)
    assume_airborne = swarm.allcfs.get_parameter('assume_airborne').value

    has_taken_off = assume_airborne
    received_first_cmd_vel = False
    msg_cmd_vel = Twist()

    def cmd_vel_callback(msg):
        nonlocal msg_cmd_vel, received_first_cmd_vel
        msg_cmd_vel = msg
        msg_is_zero = (
            msg.linear.x == 0.0
            and msg.linear.y == 0.0
            and msg.angular.z == 0.0
            and msg.linear.z == 0.0
        )
        if not msg_is_zero and not received_first_cmd_vel and msg.linear.z >= 0.0:
            received_first_cmd_vel = True

    swarm.allcfs.create_subscription(Twist, '/cmd_vel', cmd_vel_callback, 10)

    swarm.allcfs.get_logger().info(
        f'vel_mux ready for {cf.prefix}, hover height: {HOVER_HEIGHT} m, '
        f'assume_airborne={assume_airborne}'
    )

    while rclpy.ok():
        if received_first_cmd_vel and not has_taken_off:
            cf.takeoff(targetHeight=HOVER_HEIGHT, duration=TAKEOFF_DURATION)
            has_taken_off = True
            timeHelper.sleep(TAKEOFF_DURATION)

        if received_first_cmd_vel and has_taken_off:
            if msg_cmd_vel.linear.z >= 0:
                # Pre-negate yaw here to cancel out crazyflie_server's own
                # `-1.0 * degrees(...)` flip in its hover-setpoint handling
                # (crazyflie_server_py: _cmd_hover_changed) before it reaches
                # the firmware. The sim path (ros_gz_crazyflie's
                # control_services.py) passes angular.z straight through with
                # no such flip, so without this the same
                # wall_following_direction_value turns one way in Gazebo and
                # the opposite way on the real drone.
                # 这里提前把yaw取反一次,抵消crazyflie_server自己在处理hover
                # setpoint时做的那次`-1.0 * degrees(...)`取反(见
                # crazyflie_server_py的_cmd_hover_changed)。仿真那条路径
                # (ros_gz_crazyflie的control_services.py)是原样传递
                # angular.z、没有做这次取反的,所以不加这一行的话,同一个
                # wall_following_direction_value在Gazebo里和真机上转的方向
                # 会正好相反。
                cf.cmdHover(
                    vx=msg_cmd_vel.linear.x,
                    vy=msg_cmd_vel.linear.y,
                    yaw_rate=-msg_cmd_vel.angular.z,
                    z_distance=HOVER_HEIGHT,
                )
            else:
                cf.notifySetpointsStop()
                cf.land(targetHeight=LAND_HEIGHT, duration=TAKEOFF_DURATION)
                timeHelper.sleep(TAKEOFF_DURATION)
                has_taken_off = False
                received_first_cmd_vel = False

        timeHelper.sleepForRate(10)


if __name__ == '__main__':
    main()
