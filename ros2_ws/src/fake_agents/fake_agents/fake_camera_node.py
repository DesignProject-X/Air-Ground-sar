import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from custom_msgs.msg import UavDispatch


class FakeCameraNode(Node):
    """
    Simulates the camera detection pipeline.
    Starts "searching" as soon as the UAV is dispatched (mirrors the real
    camera running concurrently with flight, independent of mapping), then
    publishes a fixed target pose after a short delay to mimic YOLO
    detection latency. The delay is shorter than fake_uav_node's 3s recon
    timer so the target typically arrives while the scheduler is still in
    RECON, exercising the early-detection code path.
    """

    DETECTION_DELAY_S = 2.0

    def __init__(self):
        super().__init__('fake_camera_node')
        self._timer = None
        self._dispatched = False

        self.create_subscription(UavDispatch, '/uav/dispatch', self._on_dispatch, 10)
        self.pub = self.create_publisher(PoseStamped, '/camera/target_pose', 10)
        self.get_logger().info('[Fake Camera] Ready. Waiting for UAV dispatch...')

    def _on_dispatch(self, msg: UavDispatch):
        if self._dispatched:
            return
        self._dispatched = True
        self.get_logger().info(
            f'[Fake Camera] UAV dispatched. '
            f'Simulating target detection ({self.DETECTION_DELAY_S}s)...')
        if self._timer is not None:
            self._timer.cancel()
        self._timer = self.create_timer(self.DETECTION_DELAY_S, self._publish_target_once)

    def _publish_target_once(self):
        self._timer.cancel()
        self._dispatched = False  # reset for next mission

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
