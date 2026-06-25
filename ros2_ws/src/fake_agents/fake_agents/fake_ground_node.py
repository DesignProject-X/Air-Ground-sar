import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped


class FakeGroundNode(Node):

    def __init__(self):
        super().__init__('fake_ground_node')
        self._timer = None

        self.create_subscription(PoseStamped, '/ground/goal_pose', self._on_goal, 10)
        self.pub_reached = self.create_publisher(Bool, '/goal_reached', 10)
        self.pub_status = self.create_publisher(String, '/nav_status', 10)
        self.get_logger().info('[Fake Ground] Ready. Waiting for goal pose...')

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
