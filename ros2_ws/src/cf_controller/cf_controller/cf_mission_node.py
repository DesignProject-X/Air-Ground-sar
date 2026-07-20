#!/usr/bin/env python3
"""
CrazyFlie Mission Bridge Node
CrazyFlie 任务桥接节点

Bridges scheduler dispatch (start point + search direction) to real flight
control: Takeoff -> GoTo(start) -> start wall-following exploration ->
on camera target detection: stop exploration -> notify_setpoints_stop ->
GoTo(target) -> linger to let the mapper finish covering the surroundings
-> report recon complete via MapResult.
将 scheduler 的派发(起点+探索方向)桥接到实机飞控:起飞 -> 飞到起点 ->
开始沿墙探索 -> 相机发现目标后停止探索 -> 清除速度设定点 -> 飞向目标 ->
悬停让建图补全周边 -> 上报侦察完成(MapResult)。

Usage / 用法:
    Terminal A: ros2 launch crazyflie_ros2_multiranger_bringup wall_follower_mapper_real.launch.py
    (cf_mission_node is launched as part of that file)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Trigger
from crazyflie_interfaces.srv import Takeoff, GoTo, NotifySetpointsStop
from builtin_interfaces.msg import Duration
from custom_msgs.msg import UavDispatch, MapResult
from custom_msgs.srv import StartWallFollowing


HOVER_HEIGHT     = 0.3   # Hover/search height in meters / 悬停与探索高度(米)
                         # Kept below the maze wall height (~0.4m) so the
                         # multiranger side sensors stay pointed at the walls
                         # instead of flying over them.
                         # 保持在迷宫墙高(约0.4米)以下,确保multiranger侧向传感器能
                         # 探测到墙,而不是飞到墙顶上方。
TAKEOFF_DURATION = 3.0   # Takeoff time in seconds / 起飞用时(秒)
GOTO_DURATION    = 3.0   # Time to reach a GoTo goal / 飞向目标点用时(秒)
LINGER_SECONDS   = 5.0   # Extra hover time at target so the mapper covers
                         # surroundings before recon is reported complete
                         # 到达目标后额外悬停时间,让建图补全周边(秒)


def make_duration(seconds: float) -> Duration:
    """Convert float seconds to ROS Duration message / 将秒数转为ROS Duration消息"""
    d = Duration()
    d.sec = int(seconds)
    d.nanosec = int((seconds - int(seconds)) * 1e9)
    return d


class CfMissionNode(Node):

    def __init__(self):
        super().__init__('cf_mission_node')

        self.declare_parameter('robot_prefix', 'crazyflie_real')
        robot_prefix = self.get_parameter('robot_prefix').value
        self.prefix = '/' + robot_prefix.lstrip('/')

        # Service clients / 服务客户端
        self.takeoff_client   = self.create_client(Takeoff, self.prefix + '/takeoff')
        self.goto_client      = self.create_client(GoTo, self.prefix + '/go_to')
        self.notify_stop_client = self.create_client(
            NotifySetpointsStop, self.prefix + '/notify_setpoints_stop')
        self.start_wf_client  = self.create_client(
            StartWallFollowing, self.prefix + '/start_wall_following')
        self.stop_wf_client   = self.create_client(Trigger, self.prefix + '/stop_wall_following')

        self.get_logger().info('Waiting for flight services... / 等待飞控服务就绪...')
        for client in (self.takeoff_client, self.goto_client, self.notify_stop_client,
                       self.start_wf_client, self.stop_wf_client):
            client.wait_for_service(timeout_sec=5.0)
        self.get_logger().info('All services ready. / 所有服务就绪')

        # Latest map, cached for reporting recon complete / 缓存最新地图，用于上报侦察完成
        self.latest_map = None
        self.map_sub = self.create_subscription(
            OccupancyGrid, self.prefix + '/map', self._on_map,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       history=HistoryPolicy.KEEP_LAST))

        self.dispatch_sub = self.create_subscription(
            UavDispatch, '/uav/dispatch', self._on_dispatch, 10)
        self.target_sub = self.create_subscription(
            PoseStamped, '/camera/target_pose', self._on_target, 10)
        self.map_result_pub = self.create_publisher(MapResult, '/uav/map_result', 10)

        # Mission state / 任务状态
        self._searching = False
        self._active_timer = None
        self._start_position = None
        self._start_yaw = 0.0
        self._direction = 'right'
        self._target_pose = None

        self.get_logger().info('UAV mission node ready. Waiting for dispatch... / 就绪，等待派发...')

    def _cancel_active_timer(self):
        if self._active_timer is not None:
            self._active_timer.cancel()
            self._active_timer = None

    def _on_map(self, msg: OccupancyGrid):
        self.latest_map = msg

    def _on_dispatch(self, msg: UavDispatch):
        self.get_logger().info(
            f'Dispatch received: start=({msg.start_position.x:.2f}, {msg.start_position.y:.2f}), '
            f'direction={msg.direction}')
        self._start_position = msg.start_position
        self._start_yaw = msg.start_yaw
        self._direction = msg.direction

        self._cancel_active_timer()
        req = Takeoff.Request()
        req.group_mask = 0
        req.height = HOVER_HEIGHT
        req.duration = make_duration(TAKEOFF_DURATION)
        self.takeoff_client.call_async(req)
        self.get_logger().info(f'Takeoff command sent: height={HOVER_HEIGHT}m')

        self._active_timer = self.create_timer(TAKEOFF_DURATION + 1.0, self._goto_start)

    def _goto_start(self):
        self._cancel_active_timer()
        req = GoTo.Request()
        req.group_mask = 0
        req.relative = False
        req.goal = self._start_position
        req.yaw = self._start_yaw
        req.duration = make_duration(GOTO_DURATION)
        self.goto_client.call_async(req)
        self.get_logger().info(
            f'GoTo start sent: ({self._start_position.x:.2f}, {self._start_position.y:.2f})')

        self._active_timer = self.create_timer(GOTO_DURATION + 1.0, self._begin_search)

    def _begin_search(self):
        self._cancel_active_timer()
        req = StartWallFollowing.Request()
        req.direction = self._direction
        self.start_wf_client.call_async(req)
        self._searching = True
        self.get_logger().info(f'Wall-following search started, direction={self._direction}')

    def _on_target(self, msg: PoseStamped):
        if not self._searching:
            return
        self._searching = False
        self._target_pose = msg
        self.get_logger().info(
            f'Target detected at ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}). '
            f'Stopping search.')

        self.stop_wf_client.call_async(Trigger.Request())

        # Streaming velocity setpoints from wall-following preempt high-level
        # GoTo commands until notify_setpoints_stop is called.
        # 沿墙探索的流式速度设定点会抢占高层 GoTo 指令，必须先调用 notify_setpoints_stop 清除。
        notify_req = NotifySetpointsStop.Request()
        notify_req.remain_valid_millisecs = 100
        notify_req.group_mask = 0
        self.notify_stop_client.call_async(notify_req)

        self._cancel_active_timer()
        self._active_timer = self.create_timer(1.0, self._goto_target)

    def _goto_target(self):
        self._cancel_active_timer()
        req = GoTo.Request()
        req.group_mask = 0
        req.relative = False
        req.goal = self._target_pose.pose.position
        req.yaw = 0.0
        req.duration = make_duration(GOTO_DURATION)
        self.goto_client.call_async(req)
        self.get_logger().info(
            f'GoTo target sent: ({self._target_pose.pose.position.x:.2f}, '
            f'{self._target_pose.pose.position.y:.2f})')

        self._active_timer = self.create_timer(GOTO_DURATION + LINGER_SECONDS,
                                                self._report_recon_complete)

    def _report_recon_complete(self):
        self._cancel_active_timer()
        if self.latest_map is None:
            self.get_logger().error('No map received yet; cannot report recon complete.')
            return

        result = MapResult()
        result.map = self.latest_map
        result.confidence = 0.9
        result.frame_id = 'map'
        self.map_result_pub.publish(result)
        self.get_logger().info('Recon complete. MapResult published.')


def main(args=None):
    rclpy.init(args=args)
    node = CfMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
