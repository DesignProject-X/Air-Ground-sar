import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped
from custom_msgs.srv import SaveMap
from std_srvs.srv import Trigger


class FakeGroundNode(Node):

    def __init__(self):
        super().__init__('fake_ground_node')
        self._timer = None

        self.create_subscription(PoseStamped, '/ground/goal_pose', self._on_goal, 10)
        self.pub_reached = self.create_publisher(Bool, '/goal_reached', 10)
        self.pub_status = self.create_publisher(String, '/nav_status', 10)
        # Stand-ins for map_receiver_node's real services, so the scheduler's
        # MAP_READY step (save_map) and its "does the ground robot already
        # have a saved map" check (load_existing_map) both have something to
        # call in simulation instead of timing out. This fake never actually
        # has a saved map - keeps the simulated test sequence always going
        # through aerial_recon, matching the documented test cases.
        # map_receiver_node真实服务的替身,让调度器的MAP_READY步骤(save_map)
        # 和"机器人有没有存过地图"的检查(load_existing_map)在仿真里都有
        # 东西可调,而不是超时。这个假节点永远没有存过地图——保证仿真测试
        # 序列始终会走aerial_recon这条路,和文档里写的测试用例保持一致。
        self.create_service(SaveMap, 'save_map', self._on_save_map)
        self.create_service(Trigger, 'load_existing_map', self._on_load_existing_map)
        self.get_logger().info('[Fake Ground] Ready. Waiting for goal pose...')

    def _on_save_map(self, request, response):
        response.success = True
        response.message = 'Fake ground robot accepted the map (simulated, nothing written to disk).'
        self.get_logger().info(f'[Fake Ground] {response.message}')
        return response

    def _on_load_existing_map(self, request, response):
        response.success = False
        response.message = 'Fake ground robot never has a saved map (simulated).'
        return response

    def _on_goal(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.get_logger().info(
            f'[Fake Ground] Goal received ({x:.2f}, {y:.2f}). Navigating... (simulated 4s)')
        self.pub_status.publish(String(data='NAVIGATING'))
        # 取消上一次计时器，防止 retry 时重叠触发
        if self._timer is not None:
            self._timer.cancel()
        self._timer = self.create_timer(4.0, self._arrive)

    def _arrive(self):
        self._timer.cancel()  # 单次触发，立即取消避免重复发布
        self.get_logger().info('[Fake Ground] Target reached.')
        self.pub_status.publish(String(data='SUCCEEDED'))
        self.pub_reached.publish(Bool(data=True))


def main(args=None):
    rclpy.init(args=args)
    node = FakeGroundNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
