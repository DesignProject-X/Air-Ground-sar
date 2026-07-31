import functools
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from visualization_msgs.msg import Marker

import tf2_ros
import tf2_geometry_msgs  # noqa: F401  registers PoseStamped with tf2_ros.Buffer.transform


class GroundControllerNode(Node):

    # Nav2 has been seen reporting STATUS_SUCCEEDED (within its own
    # xy_goal_tolerance) at a position visibly far from the actual target
    # (AMCL localization jump is the leading suspect, not yet confirmed) -
    # so on both success and failure, re-check the real straight-line
    # distance ourselves and resend the same goal if it's still too far,
    # instead of trusting Nav2's own verdict blindly. Capped so a
    # persistently-unreachable target still gives up instead of looping
    # forever.
    # 观察到过Nav2报STATUS_SUCCEEDED(在它自己的xy_goal_tolerance范围内),
    # 但实际停的位置明显离目标很远(怀疑是AMCL定位跳变,尚未完全确认)——
    # 所以不管成功还是失败,都自己再核实一次真实直线距离,如果还是太远就
    # 重新发送同一个目标,而不是完全相信Nav2自己的判断。设了重试上限,
    # 避免目标一直够不到时无限重发。
    _RETRY_DISTANCE_THRESHOLD = 0.15
    _MAX_RETRIES = 2

    def __init__(self):
        super().__init__('ground_controller_node')

        self._cb_group = ReentrantCallbackGroup()
        self._goal_handle = None
        self._retry_count = 0

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

        # Diagnostic only - see _on_amcl_pose. Logs every AMCL update (pose +
        # covariance) to check whether a "Reached the goal!" that turns out
        # to be far from the actual target (per _log_target_vs_actual) lines
        # up with AMCL's own belief suddenly jumping/re-converging around
        # that time - this maze is small and its corridors may look similar
        # enough from scan data alone that AMCL's particle filter has more
        # than one plausible pose to choose between.
        # 仅用于诊断——见_on_amcl_pose。记录每一次AMCL更新(位置+协方差),
        # 用来检查"报了Reached the goal!、但实际离目标很远"(见
        # _log_target_vs_actual)是否跟AMCL自己的位置估计在那前后突然跳变/
        # 重新收敛对得上——这个迷宫比较小,走廊单看扫描数据可能长得比较像,
        # AMCL的粒子滤波器有可能在好几个差不多合理的位置估计之间摇摆。
        # AMCL publishes /amcl_pose as TRANSIENT_LOCAL (so a late subscriber,
        # e.g. RViz, immediately gets the last pose) - a default VOLATILE
        # subscription is QoS-incompatible with that and silently receives
        # nothing at all. Confirmed live: an entire test session's worth of
        # logs had zero DIAGNOSTIC amcl_pose lines despite AMCL clearly
        # running and localizing (Nav2 itself was using its pose to navigate
        # the whole time).
        # AMCL发布/amcl_pose用的是TRANSIENT_LOCAL(这样晚订阅的比如RViz也能
        # 立刻拿到最后一次位姿)——默认的VOLATILE订阅跟这个QoS不兼容,会完全
        # 收不到任何消息,而且不会报错。实测确认过:一整轮测试的日志里,
        # DIAGNOSTIC amcl_pose一行都没有,但AMCL明明在正常跑、在定位
        # (Nav2整个过程都是拿着它的位姿在导航的)。
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl_pose,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       history=HistoryPolicy.KEEP_LAST),
            callback_group=self._cb_group)

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
        self._retry_count = 0
        self._send_goal(msg)

    def _send_goal(self, pose_stamped: PoseStamped):
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
        goal.pose = pose_stamped
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.header.frame_id = 'map'

        self._pub_status.publish(String(data='NAVIGATING'))

        future = self._nav_client.send_goal_async(
            goal,
            feedback_callback=self._on_feedback,
        )
        # Bind the target this specific goal was sent for, rather than
        # reading back some shared "current target" field - a newer external
        # goal can arrive and overwrite that before this goal's own result
        # callback fires (e.g. right after being cancelled), which would
        # otherwise compare the robot's position against the WRONG target.
        # 把这次目标绑定到这个具体的请求上,而不是回调里再去读一个共享的
        # "当前目标"字段——新的外部目标有可能在这个目标自己的结果回调触发
        # 之前就到达并覆盖掉那个字段(比如这个目标刚被取消的时候),那样会拿
        # 错误的目标去比较机器人当前位置。
        future.add_done_callback(
            functools.partial(self._on_goal_response, target=pose_stamped))

    def _on_goal_response(self, future, target: PoseStamped):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('[Ground Controller] Goal rejected by Nav2.')
            self._pub_status.publish(String(data='FAILED'))
            self._pub_reached.publish(Bool(data=False))
            return

        self._goal_handle = handle
        self.get_logger().info('[Ground Controller] Goal accepted. Navigating...')
        handle.get_result_async().add_done_callback(
            functools.partial(self._on_result, target=target))

    def _on_feedback(self, feedback_msg):
        eta = feedback_msg.feedback.estimated_time_remaining.sec
        self.get_logger().debug(f'[Ground Controller] ETA: {eta}s')

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        cov = msg.pose.covariance
        # Flattened row-major 6x6 over [x, y, z, roll, pitch, yaw] - index 0
        # is xx variance, 7 is yy, 35 is yaw-yaw.
        # 按行展开的6x6矩阵,顺序是[x, y, z, roll, pitch, yaw] - 索引0是x方向
        # 方差,7是y方向,35是yaw方向。
        cov_xx = cov[0]
        cov_yy = cov[7]
        cov_yaw = cov[35]
        self.get_logger().info(
            f'[Ground Controller] DIAGNOSTIC amcl_pose=({x:.3f}, {y:.3f}) '
            f'cov_xx={cov_xx:.4f} cov_yy={cov_yy:.4f} cov_yaw={cov_yaw:.4f}')

    def _log_target_vs_actual(self, target: PoseStamped):
        try:
            t = self._tf_buffer.lookup_transform('map', 'base_link', Time())
        except tf2_ros.TransformException as ex:
            self.get_logger().error(f'[Ground Controller] TF lookup failed: {ex}')
            return None
        tx = target.pose.position.x
        ty = target.pose.position.y
        rx = t.transform.translation.x
        ry = t.transform.translation.y
        distance = math.hypot(tx - rx, ty - ry)
        self.get_logger().info(
            f'[Ground Controller] DIAGNOSTIC target=({tx:.3f}, {ty:.3f}) '
            f'actual=({rx:.3f}, {ry:.3f}) real_distance={distance:.3f}m')
        return distance

    def _on_result(self, future, target: PoseStamped):
        self._goal_handle = None
        status = future.result().status
        # _retry_count is how many retries already happened BEFORE this
        # attempt was sent, so +1 gives this attempt's own ordinal (1 = the
        # original attempt, never retried).
        # _retry_count是这次尝试发出之前已经重试过的次数,所以+1就是这次
        # 尝试本身的序号(1代表最初那次,还没重试过)。
        attempt = self._retry_count + 1
        total_attempts = self._MAX_RETRIES + 1

        # A CANCELED result means a NEWER external goal already superseded
        # this one (see _send_goal) - `target` here is this old goal's own,
        # already-stale target, so a distance/retry check against it would
        # be meaningless (and would fight the goal that's now in progress).
        # CANCELED结果说明这个目标已经被一个更新的外部目标取代了(见
        # _send_goal)——这里的`target`是这个旧目标自己的、已经过时的目标,
        # 拿它做距离/重试判断没有意义(而且会跟正在进行的新目标打架)。
        if status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn(
                f'[Ground Controller] Navigation cancelled (attempt {attempt}).')
            self._pub_status.publish(String(data='CANCELLED'))
            self._pub_reached.publish(Bool(data=False))
            return

        distance = self._log_target_vs_actual(target)

        if distance is not None and distance > self._RETRY_DISTANCE_THRESHOLD:
            if self._retry_count < self._MAX_RETRIES:
                self._retry_count += 1
                self.get_logger().warn(
                    f'[Ground Controller] Attempt {attempt}/{total_attempts}: real '
                    f'distance to target ({distance:.3f}m) exceeds '
                    f'{self._RETRY_DISTANCE_THRESHOLD}m tolerance (Nav2 status={status}). '
                    f'Retrying (attempt {attempt + 1}/{total_attempts})...')
                self._send_goal(target)
                return
            self.get_logger().error(
                f'[Ground Controller] Attempt {attempt}/{total_attempts}: real distance '
                f'to target ({distance:.3f}m) still exceeds tolerance. Giving up.')
            self._pub_status.publish(String(data='FAILED'))
            self._pub_reached.publish(Bool(data=False))
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f'[Ground Controller] Target reached (attempt {attempt}/{total_attempts}).')
            self._pub_status.publish(String(data='SUCCEEDED'))
            self._pub_reached.publish(Bool(data=True))
        else:
            self.get_logger().error(
                f'[Ground Controller] Navigation failed (status={status}, '
                f'attempt {attempt}/{total_attempts}).')
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
