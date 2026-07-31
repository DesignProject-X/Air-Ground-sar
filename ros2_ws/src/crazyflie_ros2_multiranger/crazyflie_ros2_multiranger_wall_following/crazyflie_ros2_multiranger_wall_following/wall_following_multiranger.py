#!/usr/bin/env python3

""" This simple mapper is loosely based on both the bitcraze cflib point cloud example
 https://github.com/bitcraze/crazyflie-lib-python/blob/master/examples/multiranger/multiranger_pointcloud.py
 and the webots epuck simple mapper example:
 https://github.com/cyberbotics/webots_ros2

 Originally from https://github.com/knmcguire/crazyflie_ros2_experimental/
 """

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from tf2_ros import StaticTransformBroadcaster
from std_srvs.srv import Trigger
from custom_msgs.srv import StartWallFollowing

import tf_transformations
import math
import numpy as np
from .wall_following.wall_following import WallFollowing
import time

GLOBAL_SIZE_X = 20.0
GLOBAL_SIZE_Y = 20.0
MAP_RES = 0.1

# Minimum safe side distance before handing side_range to the state machine -
# below this, back off laterally instead (see timer_callback)
# 喂给状态机之前的侧向安全距离下限 - 低于这个值就横向撤离(见timer_callback)
SIDE_RANGE_MIN = 0.1


class WallFollowingMultiranger(Node):
    def __init__(self):

        super().__init__('wall_following_multiranger')
        self.declare_parameter('robot_prefix', '/crazyflie')
        robot_prefix = self.get_parameter('robot_prefix').value
        self.declare_parameter('max_turn_rate', 0.5)
        self.max_turn_rate = self.get_parameter('max_turn_rate').value
        self.declare_parameter('max_forward_speed', 0.15)
        self.max_forward_speed = self.get_parameter('max_forward_speed').value
        # Distance to keep from the followed wall (also used as the cornering
        # radius below) - must stay well under half the corridor width or the
        # Crazyflie will clip the opposite wall / be unable to turn corners.
        # 与所跟墙面保持的距离(下方转弯时也用作转弯半径) - 必须明显小于走廊宽度的一半,
        # 否则无人机会撞到对侧墙,或者转角时转不过来。
        self.declare_parameter('reference_distance_from_wall', 0.25)
        self.reference_distance_from_wall = self.get_parameter('reference_distance_from_wall').value
        # Explicit too-close/too-far bounds for command_forward_along_wall's
        # straight-line distance hold (see wall_following.py). Previously this
        # reused reference_distance_from_wall +/- ranger_value_buffer as a
        # symmetric band, which - with buffer==reference distance - made the
        # "too close, back away" correction dead code (the band extended all
        # the way down to side_range=0). wall_too_close_distance leaves a
        # 0.1m margin above the separate hard SIDE_RANGE_MIN=0.1 safety
        # override below, giving this smooth correction real room to act
        # before that blunt guard would ever need to fire.
        # command_forward_along_wall(见wall_following.py)沿墙直飞时"太近/
        # 太远"的显式边界。之前是拿reference_distance_from_wall正负
        # ranger_value_buffer凑一个对称区间,由于buffer和参考距离刚好相等,
        # "离墙太近就背离飞"这个修正变成了死代码(区间一路延伸到
        # side_range=0)。wall_too_close_distance在下面那个独立的硬保护
        # SIDE_RANGE_MIN=0.1之上留了0.1m的余地,让这个平滑修正有真正的空间
        # 去起作用,不用等到那个粗暴保护介入。
        self.declare_parameter('wall_too_close_distance', 0.2)
        self.wall_too_close_distance = self.get_parameter('wall_too_close_distance').value
        self.declare_parameter('wall_too_far_distance', 0.4)
        self.wall_too_far_distance = self.get_parameter('wall_too_far_distance').value
        # How close the front sensor must read before a wall ahead counts as
        # "reached" and triggers a turn - kept separate from
        # reference_distance_from_wall/ranger_value_buffer (previously
        # 0.25+0.2=0.45m) since those two are also reused for side-range
        # checks unrelated to the front distance - see wall_following.py.
        # Lowered from that 0.45m default to have the drone turn closer to
        # the actual wall instead of well before reaching it, for better
        # corridor coverage in this maze.
        # 前方传感器多近才算"到墙了"、该转弯——跟reference_distance_from_wall/
        # ranger_value_buffer(以前是0.25+0.2=0.45m)分开存,因为那两个参数还
        # 被另外的侧方判断复用,跟前方距离无关——见wall_following.py。从原来
        # 0.45m默认值调低,让无人机更贴近真实墙面才转弯,而不是离墙还远就转了,
        # 这样在这个迷宫里走廊覆盖更完整。
        self.declare_parameter('front_wall_detect_distance', 0.35)
        self.front_wall_detect_distance = self.get_parameter('front_wall_detect_distance').value
        # Default direction, used until a start_wall_following call overrides
        # it. Note: this value is the *turning* direction while searching for
        # a wall, not which side the wall ends up on - 'left' makes it turn
        # left while searching, which results in hugging the wall on its
        # RIGHT side (confirmed empirically; see side_range assignment below).
        # 默认方向,在start_wall_following服务被调用前生效。注意:这个值是"搜索墙时往哪转",
        # 不是"墙在哪一侧"——'left'代表搜索时往左转,实际效果是贴着右侧的墙飞(已实测确认)。
        self.wall_following_direction = 'left'

        self.odom_subscriber = self.create_subscription(
            Odometry, robot_prefix + '/odom', self.odom_subscribe_callback, 10)
        self.ranges_subscriber = self.create_subscription(
            LaserScan, robot_prefix + '/scan', self.scan_subscribe_callback, 10)

        # add service to stop wall following and make the crazyflie land
        self.srv = self.create_service(Trigger, robot_prefix + '/stop_wall_following', self.stop_wall_following_cb)
        # add service to start wall following in a given direction, on demand
        self.start_srv = self.create_service(
            StartWallFollowing, robot_prefix + '/start_wall_following', self.start_wall_following_cb)

        self.position = [0.0, 0.0, 0.0]
        self.angles = [0.0, 0.0, 0.0]
        self.ranges = [0.0, 0.0, 0.0, 0.0]

        self.position_update = False

        self.twist_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.timer = None
        self.wall_following = None

        self.get_logger().info(f"Wall following set for crazyflie " + robot_prefix +
                               f" using the scan topic. Waiting for start_wall_following call...")

    def start_wall_following_cb(self, request, response):
        self.wall_following_direction = request.direction
        self.wall_following = WallFollowing(
                max_turn_rate=self.max_turn_rate,
                max_forward_speed=self.max_forward_speed,
                reference_distance_from_wall=self.reference_distance_from_wall,
                wall_too_close_distance=self.wall_too_close_distance,
                wall_too_far_distance=self.wall_too_far_distance,
                front_wall_detect_distance=self.front_wall_detect_distance,
                init_state=WallFollowing.StateWallFollowing.FORWARD)
        if self.timer is not None:
            self.timer.cancel()
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.get_logger().info(f'Wall following started, direction={self.wall_following_direction}')
        response.success = True
        return response

    def stop_wall_following_cb(self, request, response):
        self.get_logger().info('Stopping wall following')
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = -0.2
        msg.angular.z = 0.0
        self.twist_publisher.publish(msg)

        response.success = True

        return response

    def timer_callback(self):

        # initialize variables
        velocity_x = 0.0
        velocity_y = 0.0
        yaw_rate = 0.0
        state_wf = WallFollowing.StateWallFollowing.HOVER

        # Get Yaw
        actual_yaw_rad = self.angles[2]

        # get front and side range in meters
        right_range = self.ranges[1]
        front_range = self.ranges[2]
        left_range = self.ranges[3]

        #self.get_logger().info(f"Front range: {front_range}, Right range: {right_range}, Left range: {left_range}")

        # choose here the direction that you want the wall following to turn to
        if self.wall_following_direction == 'right':
            wf_dir = WallFollowing.WallFollowingDirection.RIGHT
            side_range = left_range
        else:
            wf_dir = WallFollowing.WallFollowingDirection.LEFT
            side_range = right_range

        time_now = self.get_clock().now().nanoseconds * 1e-9

        # get velocity commands and current state from wall following state machine
        skipped_by_guard = not (side_range > SIDE_RANGE_MIN)
        if side_range > SIDE_RANGE_MIN:
            velocity_x, velocity_y, yaw_rate, state_wf = self.wall_following.wall_follower(
                front_range, side_range, actual_yaw_rad, wf_dir, time_now)
        else:
            # Too close to the followed wall to safely hand this side_range to
            # the state machine (e.g. TURN_TO_FIND_WALL divides by side_range,
            # which blows up as it approaches 0). This used to just leave
            # velocity at zero, which never recovers on its own - with no
            # velocity commanded, side_range never changes, so it stays stuck
            # below the guard forever (reproduced in sim: froze permanently
            # mid-maze after several corners). Nudge sideways away from the
            # wall instead, using the same sign convention as
            # command_forward_along_wall's "too close" branch, and leave the
            # state machine's own state untouched so normal following resumes
            # from exactly where it left off once side_range clears the guard.
            # 离所跟的墙太近,不能安全地把这个side_range喂给状态机(比如
            # TURN_TO_FIND_WALL里会用side_range做除数,趋近0时会炸)。之前这里
            # 只是让速度停在零,但这样永远好不了——不发速度,side_range就不会
            # 变化,会一直卡在安全阈值以下(仿真里复现过:走了好几个转角后,
            # 在迷宫中间永久卡死)。改成沿用command_forward_along_wall里"太近"
            # 分支的符号约定,横向撤离,不动状态机自己的状态,side_range恢复到
            # 安全范围后,巡墙直接从原来的状态继续,不会跳步。
            direction_value = getattr(
                self.wall_following, 'wall_following_direction_value', wf_dir.value)
            velocity_y = direction_value * (self.max_forward_speed / 2.0)
            state_wf = self.wall_following.state

        # TEMP DEBUG: log every tick the reported state changes, plus a
        # periodic heartbeat, to catch the exact tick a stall starts -
        # the 0.5s-interval heartbeat alone skips ~50 ticks between samples,
        # too coarse to see which transition caused a freeze.
        if not hasattr(self, '_debug_count'):
            self._debug_count = 0
            self._last_state_wf = None
        self._debug_count += 1
        state_changed = state_wf != self._last_state_wf
        if state_changed or self._debug_count % 10 == 0:
            self.get_logger().info(
                f'{"[CHANGE] " if state_changed else ""}'
                f'state={state_wf.name} guard_skip={skipped_by_guard} '
                f'front={front_range:.3f} side={side_range:.3f} '
                f'vx={velocity_x:.2f} vy={velocity_y:.2f} yaw_rate={yaw_rate:.2f} '
                f'yaw_deg={math.degrees(actual_yaw_rad):.1f} '
                f'pos=({self.position[0]:.2f},{self.position[1]:.2f},{self.position[2]:.3f}) '
                f'fsm_state={self.wall_following.state.name}')
        self._last_state_wf = state_wf



        msg = Twist()
        msg.linear.x = velocity_x
        msg.linear.y = velocity_y
        msg.angular.z = yaw_rate
        self.twist_publisher.publish(msg)

    def odom_subscribe_callback(self, msg):
        self.position[0] = msg.pose.pose.position.x
        self.position[1] = msg.pose.pose.position.y
        self.position[2] = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        euler = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.angles[0] = euler[0]
        self.angles[1] = euler[1]
        self.angles[2] = euler[2]
        self.position_update = True

    def scan_subscribe_callback(self, msg):
        self.ranges = msg.ranges

def main(args=None):

    rclpy.init(args=args)
    wall_following_multiranger = WallFollowingMultiranger()
    rclpy.spin(wall_following_multiranger)
    rclpy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
