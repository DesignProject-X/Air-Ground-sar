import os

import cv2
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from custom_msgs.msg import MapResult, UavDispatch

DEFAULT_MAP_YAML = os.path.expanduser(
    '~/01_SAP/Air-Ground-sar/ros2_ws/src/sar_bringup/maps/maze_map.yaml')


class FakeUavNode(Node):

    def __init__(self):
        super().__init__('fake_uav_node')
        self._timer = None

        self.declare_parameter('map_yaml_path', DEFAULT_MAP_YAML)
        self.map_yaml_path = self.get_parameter('map_yaml_path').value

        self.create_subscription(UavDispatch, '/uav/dispatch', self._on_dispatch, 10)
        self.pub = self.create_publisher(MapResult, '/uav/map_result', 10)
        self.get_logger().info(
            f'[Fake UAV] Ready. Will report {self.map_yaml_path} as the recon '
            f'result. Waiting for dispatch command...')

    def _on_dispatch(self, msg: UavDispatch):
        self.get_logger().info(
            f'[Fake UAV] Dispatch received: start=({msg.start_position.x:.2f}, '
            f'{msg.start_position.y:.2f}), direction={msg.direction}. '
            f'Simulating aerial mapping (3s)...')
        if self._timer is not None:
            self._timer.cancel()
        self._timer = self.create_timer(3.0, self._publish_map_once)

    def _load_map_as_grid(self) -> OccupancyGrid:
        with open(self.map_yaml_path) as f:
            meta = yaml.safe_load(f)

        image_path = os.path.join(os.path.dirname(self.map_yaml_path), meta['image'])
        img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise FileNotFoundError(f'Could not read map image at {image_path}')

        negate = bool(meta.get('negate', 0))
        occupied_thresh = float(meta.get('occupied_thresh', 0.65))
        free_thresh = float(meta.get('free_thresh', 0.25))

        # ROS map_server convention: pixel -> occupancy fraction (0=free,
        # 1=occupied). White is free unless negate flips that.
        occ = img.astype(np.float64) / 255.0
        if not negate:
            occ = 1.0 - occ

        data = np.full(img.shape, -1, dtype=np.int8)
        data[occ > occupied_thresh] = 100
        data[occ < free_thresh] = 0
        # values in between stay -1 (unknown), matching "trinary" mode

        # PGM is top-to-bottom, OccupancyGrid.data is bottom-to-top.
        data = np.flipud(data)

        height, width = data.shape
        resolution = float(meta['resolution'])
        origin = meta.get('origin', [0.0, 0.0, 0.0])

        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.info.resolution = resolution
        grid.info.width = width
        grid.info.height = height
        grid.info.origin.position.x = float(origin[0])
        grid.info.origin.position.y = float(origin[1])
        grid.info.origin.orientation.w = 1.0
        grid.data = data.flatten().tolist()
        return grid

    def _publish_map_once(self):
        self._timer.cancel()

        try:
            grid = self._load_map_as_grid()
        except Exception as e:
            self.get_logger().error(f'[Fake UAV] Failed to load map file: {e}')
            return

        result = MapResult()
        result.map = grid
        result.confidence = 0.92
        result.frame_id = 'map'

        self.pub.publish(result)
        self.get_logger().info(
            f'[Fake UAV] MapResult published ({grid.info.width}x{grid.info.height} '
            f'@ {grid.info.resolution:.3f}m/cell, from {self.map_yaml_path}, '
            f'confidence=0.92)')


def main(args=None):
    rclpy.init(args=args)
    node = FakeUavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
