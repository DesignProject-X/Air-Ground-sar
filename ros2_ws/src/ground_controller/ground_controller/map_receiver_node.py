#!/usr/bin/env python3
"""
Map Receiver Node
地图接收节点

Runs on the ground robot itself. Receives a built map (nav_msgs/OccupancyGrid)
from the scheduler over a plain ROS service call - no SSH/file-transfer
credentials involved, the OccupancyGrid travels over the same DDS network
link as every other topic/service in this project. Writes it out as a
.pgm/.yaml pair on the robot's own local disk, then calls map_server's own
/map_server/load_map service to switch to the new map live, without
restarting anything.
跑在小车本体上。通过普通的ROS服务调用接收调度器发来的建图结果
(nav_msgs/OccupancyGrid)——不涉及SSH/文件传输账号密码,这个OccupancyGrid
走的是跟本项目其它话题/服务完全一样的DDS网络链路。把它写成本地的
.pgm/.yaml 文件,再调用 map_server 自带的 /map_server/load_map 服务,让它
切换到这份新地图,不需要重启任何东西。

Usage / 用法:
    ros2 run ground_controller map_receiver_node
"""

import math
import os

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

from custom_msgs.srv import SaveMap
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.srv import LoadMap
from std_srvs.srv import Trigger


# Standard trinary PGM convention (matches nav2_map_server's own map_saver):
# unknown cells are mid-gray, free cells are white, occupied cells are black.
# 标准的trinary PGM惯例(和nav2_map_server自己的map_saver一致):未知格子是
# 中灰色,空闲格子是白色,占据格子是黑色。
PGM_UNKNOWN = 205
PGM_FREE = 254
PGM_OCCUPIED = 0


class MapReceiverNode(Node):

    def __init__(self):
        super().__init__('map_receiver_node')

        self.declare_parameter(
            'map_yaml_path',
            os.path.expanduser('~/tb_ws/src/tb3_launcher/maps/drone_map.yaml'))
        self.declare_parameter(
            'map_pgm_path',
            os.path.expanduser('~/tb_ws/src/tb3_launcher/maps/drone_map.pgm'))
        self.declare_parameter('load_map_service', '/map_server/load_map')
        # A freshly-loaded map means AMCL has nothing localized yet - a map
        # only just became available for it to match scans against, so this
        # is the right moment to seed it, rather than requiring a manual
        # publish that's only valid once map injection has already finished.
        # 刚加载完的地图意味着AMCL还没有定位过——地图这时候才刚好可以用来
        # 匹配扫描,这正是喂初始位姿的时机,而不是要求手动发布、还得自己
        # 判断地图注入是不是已经完成了。
        self.declare_parameter('auto_publish_initial_pose', True)
        self.declare_parameter('initial_pose_x', -0.405)
        self.declare_parameter('initial_pose_y', -0.230)
        self.declare_parameter('initial_pose_yaw', 0.0)

        self.map_yaml_path = self.get_parameter('map_yaml_path').value
        self.map_pgm_path = self.get_parameter('map_pgm_path').value
        load_map_service = self.get_parameter('load_map_service').value
        self.auto_publish_initial_pose = self.get_parameter('auto_publish_initial_pose').value

        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose',
            QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE,
                       history=HistoryPolicy.KEEP_LAST))

        # SaveMap's callback blocks on the LoadMap client call completing -
        # both need to run on a reentrant group under a multi-threaded
        # executor, or the LoadMap response callback would never get a
        # chance to run while SaveMap's callback is still executing (same
        # pattern used in ground_controller_node for the Nav2 action client).
        # SaveMap的回调要阻塞等LoadMap客户端调用完成——两者都得放在可重入
        # (reentrant)分组下、配合多线程执行器用,不然SaveMap的回调还没执行完时,
        # LoadMap的响应回调永远轮不到执行(和ground_controller_node里Nav2
        # action client用的是同一个模式)。
        cb_group = ReentrantCallbackGroup()
        self.load_map_client = self.create_client(
            LoadMap, load_map_service, callback_group=cb_group)
        self.srv = self.create_service(
            SaveMap, 'save_map', self._on_save_map, callback_group=cb_group)
        # Lets the scheduler ask "do you already have a map saved from a
        # previous run" instead of that decision being baked into a launch
        # argument - the scheduler is the one that decides whether aerial
        # recon is needed, this just answers its question.
        # 让调度器可以问"你有没有之前跑保存下来的地图",而不是把这个判断写死
        # 在launch参数里——决定要不要建图是调度器的事,这里只负责回答它的问题。
        self.load_existing_srv = self.create_service(
            Trigger, 'load_existing_map', self._on_load_existing_map, callback_group=cb_group)

        self.get_logger().info(
            f'Map receiver ready. Will write to {self.map_pgm_path} and '
            f'reload via {load_map_service}.')

    def _write_pgm(self, grid):
        width = grid.info.width
        height = grid.info.height
        pixels = bytearray(width * height)

        # OccupancyGrid.data is row-major starting at the BOTTOM row (y=0),
        # PGM stores rows TOP-to-bottom - flip vertically while converting.
        # OccupancyGrid.data按行存储,第一行对应y=0(地图最下面那一行),
        # PGM图片是从上到下存储行的 - 转换时要把行顺序上下翻转。
        for row in range(height):
            src_row = height - 1 - row
            for col in range(width):
                value = grid.data[src_row * width + col]
                if value < 0:
                    pixel = PGM_UNKNOWN
                elif value <= 25:
                    pixel = PGM_FREE
                elif value >= 65:
                    pixel = PGM_OCCUPIED
                else:
                    # Our own mapper only ever emits -1/0/100, never a
                    # partial probability, but interpolate anyway in case
                    # another map source does.
                    # 我们自己的建图节点只会产出-1/0/100,不会有中间概率值,
                    # 但以防别的建图来源会有,这里还是做线性插值处理。
                    span = PGM_FREE - PGM_OCCUPIED
                    pixel = int(PGM_FREE - (value - 25) / 40.0 * span)
                pixels[row * width + col] = pixel

        with open(self.map_pgm_path, 'wb') as f:
            f.write(b'P5\n')
            f.write(f'{width} {height}\n'.encode())
            f.write(b'255\n')
            f.write(bytes(pixels))

    def _write_yaml(self, grid):
        origin = grid.info.origin.position
        with open(self.map_yaml_path, 'w') as f:
            f.write(f'image: {os.path.basename(self.map_pgm_path)}\n')
            f.write('mode: trinary\n')
            f.write(f'resolution: {grid.info.resolution}\n')
            f.write(f'origin: [{origin.x}, {origin.y}, 0.0]\n')
            f.write('negate: 0\n')
            f.write('occupied_thresh: 0.65\n')
            f.write('free_thresh: 0.25\n')

    def _on_save_map(self, request, response):
        try:
            self._write_pgm(request.map)
            self._write_yaml(request.map)
        except OSError as e:
            response.success = False
            response.message = f'Failed to write map files: {e}'
            self.get_logger().error(response.message)
            return response

        success, message = self._load_map_into_server()
        response.success = success
        response.message = (f'Map saved to {self.map_yaml_path} and loaded.'
                             if success else message)
        if success:
            self.get_logger().info(response.message)
            if self.auto_publish_initial_pose:
                self._publish_initial_pose()
        else:
            self.get_logger().error(response.message)
        return response

    def _on_load_existing_map(self, request, response):
        if not os.path.exists(self.map_yaml_path):
            response.success = False
            response.message = f'No saved map at {self.map_yaml_path}.'
            return response

        success, message = self._load_map_into_server()
        response.success = success
        response.message = (f'Loaded existing map from {self.map_yaml_path}.'
                             if success else message)
        if success:
            self.get_logger().info(response.message)
            if self.auto_publish_initial_pose:
                self._publish_initial_pose()
        else:
            self.get_logger().error(response.message)
        return response

    def _load_map_into_server(self) -> tuple[bool, str]:
        if not self.load_map_client.wait_for_service(timeout_sec=5.0):
            return False, 'map_server load_map service not available.'

        load_req = LoadMap.Request()
        # NOTE: this map_server build rejects a "file://" URI here (returns
        # RESULT_INVALID_MAP_METADATA) - a bare absolute path is required.
        # 注意:这个版本的map_server不接受这里的"file://"前缀
        # (会返回RESULT_INVALID_MAP_METADATA)- 必须传不带前缀的绝对路径。
        load_req.map_url = self.map_yaml_path
        future = self.load_map_client.call_async(load_req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        result = future.result()
        if result is None:
            return False, 'load_map service call timed out.'
        if result.result != 0:
            return False, f'load_map failed with result code {result.result}.'
        return True, ''

    def _publish_initial_pose(self):
        x = self.get_parameter('initial_pose_x').value
        y = self.get_parameter('initial_pose_y').value
        yaw = self.get_parameter('initial_pose_yaw').value

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.06853891945200942

        self.initial_pose_pub.publish(msg)
        self.get_logger().info(
            f'Auto-published initial pose: x={x} y={y} yaw={yaw} '
            f'(now that a fresh map is loaded).')


def main(args=None):
    rclpy.init(args=args)
    node = MapReceiverNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
