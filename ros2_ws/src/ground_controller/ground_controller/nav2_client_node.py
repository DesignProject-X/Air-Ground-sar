import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from visualization_msgs.msg import Marker

import tf2_ros
import tf2_geometry_msgs  # noqa: F401  registers PoseStamped with tf2_ros.Buffer.transform


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

        # Goals may arrive expressed relative to the robot (e.g. base_link,
        # for a target detected by an onboard camera) rather than already in
        # the map frame Nav2 requires - transform here instead of trusting
        # the caller to have done it, so upstream code (camera detection,
        # scheduler, etc.) can just publish poses in whatever frame they
        # naturally have them in.
        # 目标点可能是相对机器人本身坐标系发过来的(比如摄像头识别到的目标,
        # 天然就是相对base_link),不是Nav2需要的map坐标系 - 在这里统一转换,
        # 不要求上游(摄像头识别、调度器等)必须自己先转换好。
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.create_subscription(
            PoseStamped,
            '/ground/goal_pose',
            self._on_goal_pose,
            10,
            callback_group=self._cb_group,
        )

        self._pub_reached = self.create_publisher(Bool, '/goal_reached', 10)
        self._pub_status = self.create_publisher(String, '/nav_status', 10)

        # TRANSIENT_LOCAL so RViz still gets the last goal marker even if its
        # Marker display is added/subscribed after the goal was published.
        # TRANSIENT_LOCAL是为了让RViz即使在目标发布之后才添加/订阅Marker显示,
        # 也能收到最后一次发布的目标点标记。
        self._pub_marker = self.create_publisher(
            Marker, '/ground/goal_marker',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       history=HistoryPolicy.KEEP_LAST))

        self.get_logger().info('[Ground Controller] Ready. Waiting for goal pose...')

    def _publish_goal_marker(self, pose_stamped: PoseStamped):
        marker = Marker()
        marker.header = pose_stamped.header
        marker.ns = 'ground_goal'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose_stamped.pose
        marker.scale.x = 0.15
        marker.scale.y = 0.15
        marker.scale.z = 0.15
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime = Duration(seconds=0).to_msg()  # 0 = persist until replaced
        self._pub_marker.publish(marker)

    def _on_goal_pose(self, msg: PoseStamped):
        source_frame = msg.header.frame_id or 'map'
        if source_frame != 'map':
            try:
                msg = self._tf_buffer.transform(
                    msg, 'map', timeout=Duration(seconds=1.0))
            except tf2_ros.TransformException as ex:
                self.get_logger().error(
                    f'[Ground Controller] Could not transform goal from '
                    f'"{source_frame}" to "map": {ex}')
                self._pub_status.publish(String(data='FAILED'))
                self._pub_reached.publish(Bool(data=False))
                return

        x = msg.pose.position.x
        y = msg.pose.position.y
        self.get_logger().info(
            f'[Ground Controller] Goal received ({x:.2f}, {y:.2f}) in map frame '
            f'(source frame: {source_frame}). Sending to Nav2...')
        self._publish_goal_marker(msg)

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
