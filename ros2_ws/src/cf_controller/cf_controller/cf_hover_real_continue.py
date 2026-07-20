#!/usr/bin/env python3
"""
CrazyFlie Real Robot Basic Hover Node
CrazyFlie 实机原地悬停节点

Sequence: Takeoff to HOVER_HEIGHT -> hold in place for HOVER_SECONDS -> Land
流程: 起飞到 HOVER_HEIGHT -> 原地悬停 HOVER_SECONDS 秒 -> 降落

Usage / 用法:
    Terminal A: ros2 launch cf_controller real_robot_launch.py
    Terminal B: ros2 run cf_controller cf_hover_real_continue
"""

import rclpy
from rclpy.node import Node
from crazyflie_interfaces.srv import Takeoff, Land
from builtin_interfaces.msg import Duration


# Flight parameters / 飞行参数
# NOTE: this project has two different real-hardware launch files that name
# the same physical drone (same radio URI) differently:
#   - cf_controller/launch/real_robot_launch.py uses crazyswarm2's own
#     default crazyflies.yaml, which names the robot 'cf231' — this is the
#     launch file this node's usage instructions point to, so match it here.
#   - crazyflie_ros2_multiranger_bringup's wall_follower_mapper_real launch
#     uses crazyflie_real_crazyswarm2.yaml, naming it 'crazyflie_real'
#     (used by cf_mission_node, wall_following, simple_mapper).
# Make sure ROBOT_PREFIX always matches whichever launch file you actually run.
ROBOT_PREFIX     = '/cf231'  # Robot topic/service prefix / 机器人话题前缀
HOVER_HEIGHT     = 0.3       # Hover height in meters / 悬停高度(米)
TAKEOFF_DURATION = 3.0       # Takeoff time in seconds / 起飞用时(秒)
# Long hover window so you have time to visually confirm the position
# estimate is stable (no drift) before starting wall_following in another
# terminal. Ctrl+C lands immediately at any point if you don't want to wait
# out the full 5 minutes.
# 悬停时间拉长,给自己时间确认位置估计稳定(不漂移)之后,再在另一个终端启动
# 沿墙程序。随时按 Ctrl+C 都会立刻降落,不需要等满5分钟。
HOVER_SECONDS    = 300.0     # Hover time in seconds / 悬停时间(秒) - 5 minutes
LAND_DURATION    = 3.0       # Landing time in seconds / 降落用时(秒)
START_DELAY      = 3.0       # Delay before takeoff for Kalman to stabilize / 起飞前等待时间(秒)


def make_duration(seconds: float) -> Duration:
    """Convert float seconds to ROS Duration message / 将秒数转为ROS Duration消息"""
    d = Duration()
    d.sec = int(seconds)
    d.nanosec = int((seconds - int(seconds)) * 1e9)
    return d


class CfHoverReal(Node):

    def __init__(self):
        super().__init__('cf_hover_real_continue')

        # Active timer reference for cancellation / 当前定时器引用，用于取消防止重复触发
        self.active_timer = None

        # Service clients / 服务客户端
        self.takeoff_client = self.create_client(Takeoff, ROBOT_PREFIX + '/takeoff')
        self.land_client    = self.create_client(Land,    ROBOT_PREFIX + '/land')

        # Mission state flags / 任务状态标志
        self.mission_started = False
        self.mission_done    = False

        # Wait for services to be available / 等待服务就绪
        self.get_logger().info(
            f'Waiting for crazyswarm2 services under {ROBOT_PREFIX}... / 等待服务就绪...')
        takeoff_ready = self.takeoff_client.wait_for_service(timeout_sec=5.0)
        land_ready = self.land_client.wait_for_service(timeout_sec=5.0)
        if not (takeoff_ready and land_ready):
            self.get_logger().error(
                f'Service(s) under {ROBOT_PREFIX} never came up (takeoff={takeoff_ready}, '
                f'land={land_ready}). Check that ROBOT_PREFIX matches the robot name in '
                f'whichever crazyflies.yaml the launch file you used actually loaded — '
                f'commands sent to a nonexistent service silently go nowhere.')
        else:
            self.get_logger().info('All services ready. / 所有服务就绪')

        self.get_logger().info('CrazyFlie hover node started. / 悬停节点启动')
        self.get_logger().info(
            f'Will take off to {HOVER_HEIGHT}m, hover {HOVER_SECONDS}s, then land.'
        )
        self.get_logger().info(
            f'Takeoff in {START_DELAY}s. '
            f'Ensure: textured floor, sufficient light, 1m clearance. '
            f'Press Ctrl+C to cancel.'
        )

        # Start mission after delay / 延迟后启动任务
        self.active_timer = self.create_timer(START_DELAY, self.start_mission)

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

        self.get_logger().info('Mission started: takeoff. / 任务开始:起飞')
        self.call_takeoff()

        # Takeoff already commands the drone to climb to HOVER_HEIGHT and
        # hold there — no GoTo needed to "stay in place". Just wait out the
        # hover duration, then land.
        # Takeoff 指令本身就会让无人机爬升到 HOVER_HEIGHT 并原地悬停，不需要
        # 额外发 GoTo 来"保持原地"——只需等悬停时长结束后降落即可。
        self.active_timer = self.create_timer(
            TAKEOFF_DURATION + HOVER_SECONDS,
            self.finish_hover
        )
        self.get_logger().info(
            f'Hovering for {HOVER_SECONDS}s after takeoff completes, then landing.'
        )

    def finish_hover(self):
        """Hover time elapsed, land. / 悬停时间结束，降落。"""
        self.cancel_active_timer()

        if self.mission_done:
            return

        self.get_logger().info('Hover complete, landing. / 悬停完成，开始降落')
        self.call_land()
        self.mission_done = True
        self.get_logger().info('Mission complete. Press Ctrl+C to exit. / 任务结束')


def main(args=None):
    rclpy.init(args=args)
    node = CfHoverReal()

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
