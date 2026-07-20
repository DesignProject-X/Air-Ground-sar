#!/usr/bin/env python3
"""
CrazyFlie Real Robot Waypoint Navigation Node
CrazyFlie 实机航点飞行节点

Sequence: Takeoff -> Fly to 4 waypoints (absolute) -> Land
流程: 起飞 -> 依次飞向4个航点(绝对坐标) -> 降落

Usage / 用法:
    Terminal A: ros2 launch cf_controller real_robot_launch.py
    Terminal B: ros2 run cf_controller cf_waypoint_real
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from crazyflie_interfaces.srv import Takeoff, Land, GoTo
from builtin_interfaces.msg import Duration


# Flight parameters / 飞行参数
ROBOT_PREFIX     = '/cf231'  # Robot topic/service prefix / 机器人话题前缀
HOVER_HEIGHT     = 0.4       # Hover height in meters / 悬停高度(米)
TAKEOFF_DURATION = 3.0       # Takeoff time in seconds / 起飞用时(秒)
FLY_DURATION     = 2.0       # Time to reach each waypoint / 飞向每个航点用时(秒)
HOVER_SECONDS    = 3.0       # Hover time at each waypoint / 每个航点悬停时间(秒)
LAND_DURATION    = 3.0       # Landing time in seconds / 降落用时(秒)
START_DELAY      = 3.0       # Delay before takeoff for Kalman to stabilize / 起飞前等待时间(秒)


def make_duration(seconds: float) -> Duration:
    """Convert float seconds to ROS Duration message / 将秒数转为ROS Duration消息"""
    d = Duration()
    d.sec = int(seconds)
    d.nanosec = int((seconds - int(seconds)) * 1e9)
    return d


class CfWaypointReal(Node):

    def __init__(self):
        super().__init__('cf_waypoint_real')

        # Waypoints in absolute coordinates (x, y, z), origin = takeoff position
        # 航点绝对坐标(x, y, z)，以起飞点为原点
        self.waypoints = [
            (0.4, 0.0, HOVER_HEIGHT),   # Waypoint 0: forward 0.4m / 前方0.4m
            (0.4, 0.4, HOVER_HEIGHT),   # Waypoint 1: left 0.4m / 左移0.4m
            (0.0, 0.4, HOVER_HEIGHT),   # Waypoint 2: backward 0.4m / 后退0.4m
            (0.0, 0.0, HOVER_HEIGHT),   # Waypoint 3: return to origin / 回原点
        ]
        self.current_wp_index = 0

        # Active timer reference for cancellation / 当前定时器引用，用于取消防止重复触发
        self.active_timer = None

        # Subscribe to pose for current position / 订阅位置话题获取当前坐标
        self.pose_sub = self.create_subscription(
            PoseStamped,
            ROBOT_PREFIX + '/pose',
            self.pose_callback,
            10
        )

        # Service clients / 服务客户端
        self.takeoff_client = self.create_client(Takeoff, ROBOT_PREFIX + '/takeoff')
        self.land_client    = self.create_client(Land,    ROBOT_PREFIX + '/land')
        self.goto_client    = self.create_client(GoTo,    ROBOT_PREFIX + '/go_to')

        # Current position / 当前位置
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0

        # Mission state flags / 任务状态标志
        self.mission_started = False
        self.mission_done    = False

        # Wait for services to be available / 等待服务就绪
        self.get_logger().info('Waiting for crazyswarm2 services... / 等待服务就绪...')
        self.takeoff_client.wait_for_service(timeout_sec=5.0)
        self.land_client.wait_for_service(timeout_sec=5.0)
        self.goto_client.wait_for_service(timeout_sec=5.0)
        self.get_logger().info('All services ready. / 所有服务就绪')

        self.get_logger().info('CrazyFlie waypoint node started. / 航点飞行节点启动')
        self.get_logger().info(f'Waypoints: {self.waypoints}')
        self.get_logger().info(
            f'Takeoff in {START_DELAY}s. '
            f'Ensure: textured floor, sufficient light, 1m clearance. '
            f'Press Ctrl+C to cancel.'
        )

        # Start mission after delay / 延迟后启动任务
        self.active_timer = self.create_timer(START_DELAY, self.start_mission)

    def pose_callback(self, msg: PoseStamped):
        """Update current position / 更新当前位置"""
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        self.current_z = msg.pose.position.z

    def cancel_active_timer(self):
        """
        Cancel the current timer to prevent repeated triggering.
        取消当前定时器，防止重复触发。
        """
        if self.active_timer is not None:
            self.active_timer.cancel()
            self.active_timer = None

    def call_takeoff(self):
        """Send takeoff command / 发送起飞指令"""
        req = Takeoff.Request()
        req.group_mask = 0
        req.height = HOVER_HEIGHT
        req.duration = make_duration(TAKEOFF_DURATION)
        self.takeoff_client.call_async(req)
        self.get_logger().info(
            f'Takeoff command sent: height={HOVER_HEIGHT}m, duration={TAKEOFF_DURATION}s'
        )

    def call_goto(self, x, y, z):
        """
        Send go_to command using absolute coordinates.
        发送飞行指令，使用绝对坐标(以起飞点为原点)。
        """
        req = GoTo.Request()
        req.group_mask = 0
        req.relative = False   # Absolute coordinates / 绝对坐标
        req.goal.x = float(x)
        req.goal.y = float(y)
        req.goal.z = float(z)
        req.yaw = 0.0
        req.duration = make_duration(FLY_DURATION)
        self.goto_client.call_async(req)
        self.get_logger().info(
            f'GoTo command sent: ({x}, {y}, {z}), duration={FLY_DURATION}s'
        )

    def call_land(self):
        """Send landing command / 发送降落指令"""
        req = Land.Request()
        req.group_mask = 0
        req.height = 0.05
        req.duration = make_duration(LAND_DURATION)
        self.land_client.call_async(req)
        self.get_logger().info(f'Land command sent: duration={LAND_DURATION}s')

    def start_mission(self):
        """Mission entry point, triggered once after START_DELAY / 任务入口，延迟后触发一次"""
        self.cancel_active_timer()

        if self.mission_started:
            return
        self.mission_started = True

        self.get_logger().info('Mission started. / 任务开始')
        self.call_takeoff()

        # Schedule first waypoint after takeoff completes / 起飞完成后飞第一个航点
        self.active_timer = self.create_timer(
            TAKEOFF_DURATION + 1.0,
            self.fly_next_waypoint
        )
        

    def fly_next_waypoint(self):
        """
        Fly to the next waypoint. When all waypoints are done, land.
        飞向下一个航点，全部完成后降落。
        """
        self.cancel_active_timer()

        # All waypoints completed, land / 所有航点完成，降落
        if self.current_wp_index >= len(self.waypoints):
            self.get_logger().info('All waypoints done, landing. / 所有航点完成，开始降落')
            self.call_land()
            self.mission_done = True
            self.get_logger().info('Mission complete. Press Ctrl+C to exit. / 任务结束')
            return

        # Send go_to command for current waypoint / 发送当前航点的飞行指令
        wp = self.waypoints[self.current_wp_index]
        self.get_logger().info(
            f'Flying to waypoint {self.current_wp_index}/{len(self.waypoints)-1}: {wp}'
        )
        self.call_goto(wp[0], wp[1], wp[2])
        self.current_wp_index += 1

        # Wait for fly + hover time, then go to next waypoint / 等待飞行+悬停时间后飞下一个
        wait_time = FLY_DURATION + HOVER_SECONDS
        self.get_logger().info(
            f'Waiting {wait_time}s (fly {FLY_DURATION}s + hover {HOVER_SECONDS}s)'
        )
        self.active_timer = self.create_timer(wait_time, self.fly_next_waypoint)


def main(args=None):
    rclpy.init(args=args)
    node = CfWaypointReal()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted, sending land command. / 收到中断，发送降落指令')
        node.cancel_active_timer()
        node.call_land()
        rclpy.spin_once(node, timeout_sec=1.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
