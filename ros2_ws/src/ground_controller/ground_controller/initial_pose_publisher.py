#!/usr/bin/env python3
"""
Ground robot initial pose publisher
小车初始位姿发布节点

Publishes a one-shot PoseWithCovarianceStamped on /initialpose so AMCL (or
slam_toolbox's localization mode) seeds its filter at the ground robot's
actual physical starting position, instead of requiring someone to click
"2D Pose Estimate" in RViz by hand every run.

x/y/yaw are in the SAME "map" frame the saved maze map uses - that frame is
centered on the maze itself (the drone's map->odom transform was anchored at
(0,0,0), and the maze was built centered on that point), so x/y here should
be how far the ground robot's actual starting spot is from the maze's
geometric center (in meters), and yaw is the compass direction it's facing
when placed (0 rad = facing the map frame's +x direction, increasing
counter-clockwise) - not the map's pixel origin, which is a fixed image
metadata value unrelated to where any robot starts.
x/y/yaw 用的是保存地图时那套"map"坐标系,这个坐标系以迷宫本身为中心(无人机的
map->odom变换锚定在(0,0,0),迷宫本身也是围绕这个点建的),所以这里的x/y应该填
"小车实际摆放的起点,距离迷宫几何中心多远"(单位米),yaw是摆放时车头朝向
(0弧度=朝向map坐标系的+x方向,逆时针为正)——不是地图的像素原点,那只是图片
的固定元数据,跟机器人放哪没关系。

Usage / 用法:
    ros2 run ground_controller initial_pose_publisher --ros-args \\
        -p x:=0.5 -p y:=-0.3 -p yaw:=1.57
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


class InitialPosePublisher(Node):

    def __init__(self):
        super().__init__('initial_pose_publisher')

        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_seconds', 3.0)

        self.x = self.get_parameter('x').value
        self.y = self.get_parameter('y').value
        self.yaw = self.get_parameter('yaw').value
        self.frame_id = self.get_parameter('frame_id').value
        publish_seconds = self.get_parameter('publish_seconds').value

        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        # /initialpose isn't latched and AMCL may not have subscribed yet the
        # instant this node starts, so keep publishing for a few seconds
        # instead of firing once - this guarantees at least one message
        # lands after AMCL actually comes up, regardless of start order.
        # /initialpose 不是latched话题,这个节点刚起来时AMCL不一定已经订阅上了,
        # 所以持续发几秒而不是只发一次——不管谁先启动,都能保证AMCL收到至少一条。
        self.end_time = self.get_clock().now().nanoseconds + int(publish_seconds * 1e9)
        self.timer = self.create_timer(0.5, self.publish_pose)

        self.get_logger().info(
            f'Publishing initial pose x={self.x} y={self.y} yaw={self.yaw}rad '
            f'in frame "{self.frame_id}" for {publish_seconds}s... '
            f'/ 发布初始位姿中...')

    def publish_pose(self):
        if self.get_clock().now().nanoseconds > self.end_time:
            self.get_logger().info('Done publishing initial pose. / 初始位姿发布完成')
            self.timer.cancel()
            return

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        # Same covariance RViz's "2D Pose Estimate" tool publishes by default
        # 和RViz"2D Pose Estimate"工具默认发布的协方差一致
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.06853891945200942

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
