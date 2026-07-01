import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


class GroundControllerNode(Node):

    def __init__(self):
        super().__init__('ground_controller_node')

        self._cb_group = ReentrantCallbackGroup()
        self._goal_handle = None

        self._nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self._cb_group,
        )

        self.create_subscription(
            PoseStamped,
            '/ground/goal_pose',
            self._on_goal_pose,
            10,
            callback_group=self._cb_group,
        )

        self._pub_reached = self.create_publisher(Bool, '/goal_reached', 10)
        self._pub_status = self.create_publisher(String, '/nav_status', 10)

        self.get_logger().info('[Ground Controller] Ready. Waiting for goal pose...')

    def _on_goal_pose(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.get_logger().info(
            f'[Ground Controller] Goal received ({x:.2f}, {y:.2f}). Sending to Nav2...')

        if self._goal_handle is not None:
            self.get_logger().info('[Ground Controller] Cancelling previous goal.')
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                '[Ground Controller] navigate_to_pose action server not available.')
            self._pub_status.publish(String(data='FAILED'))
            self._pub_reached.publish(Bool(data=False))
            return

        goal = NavigateToPose.Goal()
        goal.pose = msg
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.header.frame_id = 'map'

        self._pub_status.publish(String(data='NAVIGATING'))

        future = self._nav_client.send_goal_async(
            goal,
            feedback_callback=self._on_feedback,
        )
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('[Ground Controller] Goal rejected by Nav2.')
            self._pub_status.publish(String(data='FAILED'))
            self._pub_reached.publish(Bool(data=False))
            return

        self._goal_handle = handle
        self.get_logger().info('[Ground Controller] Goal accepted. Navigating...')
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_feedback(self, feedback_msg):
        eta = feedback_msg.feedback.estimated_time_remaining.sec
        self.get_logger().debug(f'[Ground Controller] ETA: {eta}s')

    def _on_result(self, future):
        self._goal_handle = None
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('[Ground Controller] Target reached.')
            self._pub_status.publish(String(data='SUCCEEDED'))
            self._pub_reached.publish(Bool(data=True))
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('[Ground Controller] Navigation cancelled.')
            self._pub_status.publish(String(data='CANCELLED'))
            self._pub_reached.publish(Bool(data=False))
        else:
            self.get_logger().error(
                f'[Ground Controller] Navigation failed (status={status}).')
            self._pub_status.publish(String(data='FAILED'))
            self._pub_reached.publish(Bool(data=False))


def main(args=None):
    rclpy.init(args=args)
    node = GroundControllerNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
