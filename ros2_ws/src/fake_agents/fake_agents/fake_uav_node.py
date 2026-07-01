import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid
from custom_msgs.msg import MapResult


class FakeUavNode(Node):

    def __init__(self):
        super().__init__('fake_uav_node')
        self._timer = None

        self.create_subscription(Bool, '/uav/dispatch', self._on_dispatch, 10)
        self.pub = self.create_publisher(MapResult, '/uav/map_result', 10)
        self.get_logger().info('[Fake UAV] Ready. Waiting for dispatch command...')

    def _on_dispatch(self, msg: Bool):
        if not msg.data:
            return
        self.get_logger().info('[Fake UAV] Dispatch received. Simulating aerial mapping (3s)...')
        if self._timer is not None:
            self._timer.cancel()
        self._timer = self.create_timer(3.0, self._publish_map_once)

    def _publish_map_once(self):
        self._timer.cancel()

        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.info.resolution = 0.05
        grid.info.width = 20
        grid.info.height = 20
        grid.info.origin.position.x = 0.0
        grid.info.origin.position.y = 0.0
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * (20 * 20)

        result = MapResult()
        result.map = grid
        result.confidence = 0.92
        result.frame_id = 'map'

        self.pub.publish(result)
        self.get_logger().info(
            '[Fake UAV] MapResult published (20x20 grid, confidence=0.92)')


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
