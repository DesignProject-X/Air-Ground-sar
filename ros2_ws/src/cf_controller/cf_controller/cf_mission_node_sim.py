#!/usr/bin/env python3
"""
Simulated UAV Mission Bridge Node
仿真无人机任务桥接节点

Bridges the scheduler's dispatch/recon-result protocol to the simulated
Crazyflie + wall_following + simple_mapper stack, mirroring what
cf_mission_node.py does for real hardware - adapted for two differences:

1. The sim stack has no crazyflie_interfaces Takeoff/GoTo services (control
   is Twist + /odom feedback only) - flight itself (climb, handoff to
   wall_following) is handled by cf_hover_sim.py running alongside this
   node in its dispatch-driven mode. This node only handles: caching the
   map, waiting, faking target detection, stopping wall_following, and
   reporting the MapResult.
2. Neither the real nor the simulated drone carries a camera - target
   detection is always done by the external ground camera watching the
   ground robot's ArUco marker, which has nothing to correlate against
   during a pure aerial-recon flight (no ground robot moving yet). So
   instead of waiting for a real /camera/target_pose the way
   cf_mission_node.py does on real hardware, this node fakes one after a
   fixed mapping duration - the same idea as fake_agents/
   fake_camera_node.py, just triggered by actual simulated wall-following
   progress instead of a flat delay from dispatch.
仿真版真机 cf_mission_node.py 的翻版,桥接调度器的派发/建图结果协议到
仿真的 Crazyflie + 巡墙 + 建图这套东西,适配两个差异:

1. 仿真这边没有 crazyflie_interfaces 的 Takeoff/GoTo 服务(只能靠 Twist +
   /odom 反馈控制)——起飞爬升、交接给巡墙节点这部分由跟这个节点一起跑的
   cf_hover_sim.py(派发驱动模式)负责。这个节点只管:缓存地图、等待、
   伪造目标检测、喊停巡墙、上报 MapResult。
2. 不管真机还是仿真机,无人机本身都不带摄像头——目标检测始终是外置地面
   相机在看地面机器人身上的 ArUco 标记,纯航拍建图阶段(地面机器人还没
   动)根本没有东西可以拿来对应。所以这个节点不像真机的 cf_mission_node.py
   那样等一个真实的 /camera/target_pose,而是固定建图时长之后自己伪造一个——
   跟 fake_agents/fake_camera_node.py 是同一个思路,只是触发时机改成跟着
   仿真巡墙的实际进度走,而不是派发后一个死延时。

Usage / 用法:
    ros2 launch cf_controller sim_uav_recon.launch.py
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from custom_msgs.msg import UavDispatch, MapResult

ROBOT_PREFIX = '/crazyflie'


class CfMissionNodeSim(Node):

    def __init__(self):
        super().__init__('cf_mission_node_sim')

        # Tune per how much map coverage/observation time a given test
        # actually needs.
        # 具体测试需要多少覆盖范围/观察时间,可以调这个参数。
        self.declare_parameter('mapping_duration_sec', 120.0)
        self.declare_parameter('fake_target_x', -0.5)
        self.declare_parameter('fake_target_y', 0.4)
        # Set to False when a real ground camera (target_detector_node) is
        # also running - e.g. sim UAV mapping + real ground robot + real
        # camera. Publishing a fake /camera/target_pose here would race the
        # real camera's detection and could make the scheduler skip
        # WAITING_TARGET before the real camera ever gets a chance to fire.
        # With this off, this node only ever reports the map; target
        # detection is left entirely to whatever's actually subscribed to
        # /uav/dispatch and publishing /camera/target_pose.
        # 有真实地面相机(target_detector_node)一起跑的时候(比如:仿真无人机
        # 建图 + 真实地面机器人 + 真实相机)要把这个设成False。这里再发一个假的
        # /camera/target_pose,会跟真实相机的检测抢跑,可能导致调度器在真实
        # 相机还没来得及触发之前就提前跳过了WAITING_TARGET。关掉之后,这个
        # 节点就只负责上报地图,目标检测完全交给真正在监听/uav/dispatch、
        # 发布/camera/target_pose的那个东西。
        self.declare_parameter('fake_target_signal', True)

        self.mapping_duration_sec = self.get_parameter('mapping_duration_sec').value
        self.fake_target_x = self.get_parameter('fake_target_x').value
        self.fake_target_y = self.get_parameter('fake_target_y').value
        self.fake_target_signal = self.get_parameter('fake_target_signal').value

        self.latest_map = None
        self._timer = None

        self.create_subscription(UavDispatch, '/uav/dispatch', self._on_dispatch, 10)
        self.create_subscription(
            OccupancyGrid, ROBOT_PREFIX + '/map', self._on_map, 10)

        self.target_pub = self.create_publisher(PoseStamped, '/camera/target_pose', 10)
        self.result_pub = self.create_publisher(MapResult, '/uav/map_result', 10)
        self.stop_wall_follow_client = self.create_client(
            Trigger, ROBOT_PREFIX + '/stop_wall_following')

        self.get_logger().info(
            '[Sim UAV Mission] Ready. Waiting for /uav/dispatch...')

    def _on_map(self, msg: OccupancyGrid):
        self.latest_map = msg

    def _on_dispatch(self, msg: UavDispatch):
        if self._timer is not None:
            self._timer.cancel()
        self.get_logger().info(
            f'[Sim UAV Mission] Dispatch received (direction={msg.direction}). '
            f'Simulating aerial mapping ({self.mapping_duration_sec}s)...')
        self._timer = self.create_timer(self.mapping_duration_sec, self._on_mapping_done)

    def _on_mapping_done(self):
        self._timer.cancel()

        if self.fake_target_signal:
            # Fake target detection - see module docstring / fake_target_signal
            # for why this isn't a real camera reading.
            target = PoseStamped()
            target.header.frame_id = 'map'
            target.pose.position.x = self.fake_target_x
            target.pose.position.y = self.fake_target_y
            target.pose.orientation.w = 1.0
            self.target_pub.publish(target)
            self.get_logger().info(
                f'[Sim UAV Mission] Simulated target detection at '
                f'({self.fake_target_x:.2f}, {self.fake_target_y:.2f}). '
                f'Published to /camera/target_pose.')
        else:
            self.get_logger().info(
                '[Sim UAV Mission] fake_target_signal is off - not publishing '
                '/camera/target_pose. Waiting for the real camera to detect '
                'the target instead.')

        if self.stop_wall_follow_client.service_is_ready():
            self.stop_wall_follow_client.call_async(Trigger.Request())
        else:
            self.get_logger().warn(
                '[Sim UAV Mission] stop_wall_following service not ready - '
                'drone will keep circling, but the recon result is reported anyway.')

        if self.latest_map is None:
            self.get_logger().error(
                '[Sim UAV Mission] No map received yet from simple_mapper_multiranger - '
                'cannot report MapResult.')
            return

        result = MapResult()
        result.map = self.latest_map
        result.confidence = 0.9
        result.frame_id = 'map'
        self.result_pub.publish(result)
        self.get_logger().info(
            f'[Sim UAV Mission] MapResult published '
            f'({self.latest_map.info.width}x{self.latest_map.info.height} '
            f'@ {self.latest_map.info.resolution:.3f}m/cell).')


def main(args=None):
    rclpy.init(args=args)
    node = CfMissionNodeSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
