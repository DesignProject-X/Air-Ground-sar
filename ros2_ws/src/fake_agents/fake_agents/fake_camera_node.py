import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped


class FakeCameraNode(Node):
    """
    Simulates the camera detection pipeline.
    Starts "searching" once a map is injected, then publishes a fixed
    target pose after a short delay to mimic YOLO detection latency.
    """

    DETECTION_DELAY_S = 5.0

    def __init__(self):
        super().__init__('fake_camera_node')
        self._timer = None
        self._map_received = False

        self.create_subscription(OccupancyGrid, '/map', self._on_map, 10)
        self.pub = self.create_publisher(PoseStamped, '/camera/target_pose', 10)
        self.get_logger().info('[Fake Camera] Ready. Waiting for map...')

    def _on_map(self, msg: OccupancyGrid):
        if self._map_received:
            return
        self._map_received = True
        self.get_logger().info(
            f'[Fake Camera] Map received ({msg.info.width}x{msg.info.height}). '
            f'Simulating target detection ({self.DETECTION_DELAY_S}s)...')
        if self._timer is not None:
            self._timer.cancel()
        self._timer = self.create_timer(self.DETECTION_DELAY_S, self._publish_target_once)

    def _publish_target_once(self):
        self._timer.cancel()
        self._map_received = False  # reset for next mission

        target = PoseStamped()
        target.header.frame_id = 'map'
        target.pose.position.x = 0.8
        target.pose.position.y = 0.6
        target.pose.orientation.w = 1.0

        self.pub.publish(target)
        self.get_logger().info(
            '[Fake Camera] Target detected at (0.80, 0.60). '
            'Published to /camera/target_pose.')


def main(args=None):
    rclpy.init(args=args)
    node = FakeCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
