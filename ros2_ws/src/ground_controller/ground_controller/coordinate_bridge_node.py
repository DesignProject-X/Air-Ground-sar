#!/usr/bin/env python3
"""
Coordinate Bridge Node
坐标桥接节点

Converts the UAV camera's target detection — published relative to the
ArUco marker mounted on the ground robot — into the ground robot's own map
frame, using tf2: a static base_link -> aruco_marker transform (physically
measured mounting offset) combined with the ground robot's own live
localization (map -> odom -> base_link, already required for Nav2 to
function). Publishes the result on /camera/target_pose, which the
scheduler already expects.
把无人机相机测出的目标位置(相对贴在小车上的 ArUco 标记)转换到小车自己的
地图系:用一个静态的 base_link -> aruco_marker 变换(需要实际测量安装偏移)
加上小车自己已经在跑的实时定位(map -> odom -> base_link,Nav2 本来就需要
这条),转换后发布到 /camera/target_pose,scheduler 已经在等这个话题。

Usage / 用法:
    ros2 run ground_controller coordinate_bridge_node
"""

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import (
    Buffer, ConnectivityException, ExtrapolationException,
    LookupException, StaticTransformBroadcaster, TransformListener,
)
import tf2_geometry_msgs  # noqa: F401  registers PoseStamped support on Buffer.transform()
from tf_transformations import quaternion_from_euler


class CoordinateBridgeNode(Node):

    def __init__(self):
        super().__init__('coordinate_bridge_node')

        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('marker_frame_id', 'aruco_marker')
        self.declare_parameter('map_frame_id', 'map')
        self.declare_parameter('target_pose_raw_topic', '/camera/target_pose_raw')
        self.declare_parameter('target_pose_topic', '/camera/target_pose')
        self.declare_parameter('transform_timeout_sec', 0.5)

        # Marker mounting offset relative to base_link. This is a ONE-TIME
        # PHYSICAL MEASUREMENT on the real ground robot (where the ArUco
        # marker is bolted/taped relative to the robot's own base_link
        # origin) — the all-zero default is almost certainly wrong and
        # MUST be overridden via launch params once measured.
        self.declare_parameter('marker_offset_x', 0.0)
        self.declare_parameter('marker_offset_y', 0.0)
        self.declare_parameter('marker_offset_z', 0.0)
        self.declare_parameter('marker_offset_roll', 0.0)
        self.declare_parameter('marker_offset_pitch', 0.0)
        self.declare_parameter('marker_offset_yaw', 0.0)

        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.marker_frame_id = self.get_parameter('marker_frame_id').value
        self.map_frame_id = self.get_parameter('map_frame_id').value
        self.transform_timeout_sec = self.get_parameter('transform_timeout_sec').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self._publish_static_marker_transform()

        self.create_subscription(
            PoseStamped, self.get_parameter('target_pose_raw_topic').value,
            self._on_target_pose_raw, 10)
        self.pub = self.create_publisher(
            PoseStamped, self.get_parameter('target_pose_topic').value, 10)

        self.get_logger().info(
            f'Coordinate bridge ready: {self.marker_frame_id} -> {self.map_frame_id} '
            f'(via {self.base_frame_id})')

    def _publish_static_marker_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame_id
        t.child_frame_id = self.marker_frame_id
        t.transform.translation.x = self.get_parameter('marker_offset_x').value
        t.transform.translation.y = self.get_parameter('marker_offset_y').value
        t.transform.translation.z = self.get_parameter('marker_offset_z').value
        q = quaternion_from_euler(
            self.get_parameter('marker_offset_roll').value,
            self.get_parameter('marker_offset_pitch').value,
            self.get_parameter('marker_offset_yaw').value)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.static_broadcaster.sendTransform(t)

    def _on_target_pose_raw(self, msg: PoseStamped):
        if msg.header.frame_id != self.marker_frame_id:
            self.get_logger().warn(
                f'Ignoring target pose with unexpected frame_id '
                f'"{msg.header.frame_id}" (expected "{self.marker_frame_id}").')
            return

        # Look the transform up at the latest available time rather than at
        # the detection's own stamp. Asking for the exact stamp is what the
        # timestamp field is for, but it only works while detections keep
        # arriving faster than tf2's ~10s buffer window - and detections here
        # are sparse (measured live at one every ~17.8s, since most frames
        # produce no marker+target pair). Combined with this callback running
        # one message behind, every lookup asked for a stamp ~17.8s old, i.e.
        # already aged out of the buffer, and failed with "extrapolation into
        # the past" - not once did a target reach the scheduler, which then
        # sat in WAITING_TARGET until it timed out.
        # Using the latest transform is sound here rather than merely
        # convenient: the ground robot has not been dispatched yet while the
        # scheduler waits for a target, so map -> base_link is not changing,
        # and base_link -> aruco_marker is static. Whichever instant is used,
        # the answer is the same.
        # 按"最新可用"而不是检测自己的时间戳去查变换。用精确时间戳本来才是
        # 时间戳字段的用途,但那要求检测的间隔一直小于 tf2 约 10 秒的缓存窗口
        # ——而这里的检测很稀疏(实测约 17.8 秒才有一条,因为大多数帧凑不齐
        # marker+目标)。再叠加这个回调总是慢一拍,导致每次查的都是约 17.8 秒
        # 前的时间戳,早就滑出缓存了,一律以 "extrapolation into the past"
        # 失败——目标一次都没送到调度器,它就一直卡在 WAITING_TARGET 直到超时。
        # 这里用最新变换不只是图省事,是确实成立:调度器等目标期间地面机器人
        # 还没有被派发,map -> base_link 并不会变,而 base_link -> aruco_marker
        # 本身就是静态的。取哪一时刻,结果都一样。
        lookup = PoseStamped()
        lookup.header.frame_id = msg.header.frame_id
        lookup.header.stamp = Time().to_msg()  # 0 = latest available
        lookup.pose = msg.pose
        try:
            transformed = self.tf_buffer.transform(
                lookup, self.map_frame_id,
                timeout=Duration(seconds=self.transform_timeout_sec))
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(
                f'Could not transform target pose into "{self.map_frame_id}": {e}. '
                f'Is the ground robot\'s localization (map -> {self.base_frame_id}) running?',
                throttle_duration_sec=2.0)
            return
        # Report the detection's own time, not the lookup sentinel.
        # 上报检测自身的时间,而不是查表用的那个哨兵值。
        transformed.header.stamp = msg.header.stamp

        # The target sits on the ground by definition - the z that falls out
        # of the marker-relative depth backprojection just reflects how high
        # the marker is mounted above the ground plus camera/depth noise, not
        # anything meaningful about the target itself. Nav2 only drives to
        # (x, y, yaw) anyway, so pin z to the ground instead of passing that
        # noise through.
        # 目标本来就是在地面上的——从marker坐标反投影出来的z,反映的只是marker
        # 装在离地多高、加上相机/深度噪声,并不是目标本身有意义的信息。反正
        # Nav2 也只看(x, y, yaw)去导航,与其把这份噪声传下去,不如直接把z
        # 钉在地面上。
        transformed.pose.position.z = 0.0

        self.pub.publish(transformed)
        self.get_logger().info(
            f'Target pose bridged to {self.map_frame_id}: '
            f'x={transformed.pose.position.x:.2f} y={transformed.pose.position.y:.2f} '
            f'z={transformed.pose.position.z:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = CoordinateBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
