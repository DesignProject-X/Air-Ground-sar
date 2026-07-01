import rclpy
from rclpy.node import Node
from enum import Enum, auto

from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from custom_msgs.msg import TaskCommand, MapResult


class State(Enum):
    IDLE = auto()
    RECON = auto()
    MAP_READY = auto()
    WAITING_TARGET = auto()
    NAVIGATING = auto()
    DONE = auto()
    RETRY = auto()


class SchedulerNode(Node):

    MAX_RETRIES = 3
    RECON_TIMEOUT_S = 60
    WAITING_TARGET_TIMEOUT_S = 120
    NAV_TIMEOUT_S = 120

    def __init__(self):
        super().__init__('scheduler_node')

        self.state = State.IDLE
        self.task_cmd: TaskCommand | None = None
        self.task_names: list[str] = []
        self.map_result: MapResult | None = None
        self.target_pose: PoseStamped | None = None
        self.retry_count = 0
        self.tick_counter = 0

        self.create_subscription(TaskCommand, '/scheduler/task_command',
                                 self._on_task_command, 10)
        self.create_subscription(MapResult, '/uav/map_result',
                                 self._on_map_result, 10)
        self.create_subscription(PoseStamped, '/camera/target_pose',
                                 self._on_camera_target, 10)
        self.create_subscription(Bool, '/goal_reached',
                                 self._on_goal_reached, 10)
        self.create_subscription(String, '/nav_status',
                                 self._on_nav_status, 10)

        self.pub_uav = self.create_publisher(Bool, '/uav/dispatch', 10)
        self.pub_goal = self.create_publisher(PoseStamped, '/ground/goal_pose', 10)
        self.pub_map = self.create_publisher(OccupancyGrid, '/map', 10)
        self.pub_feedback = self.create_publisher(String, '/planner/feedback', 10)

        self.create_timer(1.0, self._tick)

        self.get_logger().info('Scheduling Layer started. Waiting for task command...')

    def _on_task_command(self, msg: TaskCommand):
        if self.state != State.IDLE:
            self.get_logger().warn(
                f'New command received but current state is {self.state.name}. Ignored.')
            return
        self.task_cmd = msg
        self.task_names = [t.task for t in msg.sequence]
        self.retry_count = 0
        self.get_logger().info(
            f'Task command received: intent={msg.intent}, goal_zone={msg.goal_zone}, '
            f'sequence={self.task_names}')

        SUPPORTED = {'aerial_recon', 'map_injection', 'navigate_to_target'}
        for name in self.task_names:
            if name not in SUPPORTED:
                self.get_logger().warn(f'Task [{name}] is not yet supported by the scheduler.')

        self._transition(State.RECON)

    def _on_map_result(self, msg: MapResult):
        if self.state != State.RECON:
            return
        self.map_result = msg
        self.get_logger().info(
            f'Map result received: confidence={msg.confidence:.2f}, frame={msg.frame_id}')
        self._transition(State.MAP_READY)

    def _on_camera_target(self, msg: PoseStamped):
        if self.state != State.WAITING_TARGET:
            return
        self.target_pose = msg
        self.get_logger().info(
            f'Camera target received: x={msg.pose.position.x:.2f}, '
            f'y={msg.pose.position.y:.2f}')
        self._transition(State.NAVIGATING)

    def _on_goal_reached(self, msg: Bool):
        if self.state != State.NAVIGATING:
            return
        if msg.data:
            self.get_logger().info('Ground robot reached target. Mission complete.')
            self._transition(State.DONE)
        else:
            self.get_logger().warn('goal_reached=False. Triggering retry.')
            self._transition(State.RETRY)

    def _on_nav_status(self, msg: String):
        if self.state != State.NAVIGATING:
            return
        status = msg.data.upper()
        self.get_logger().info(f'Navigation status: {status}')
        if status in ('FAILED', 'ABORTED', 'CANCELLED'):
            self.get_logger().warn(f'Navigation failed ({status}). Triggering retry.')
            self._transition(State.RETRY)

    def _tick(self):
        self.tick_counter += 1

        if self.state == State.RECON:
            if self.tick_counter >= self.RECON_TIMEOUT_S:
                self.get_logger().warn('RECON timeout. Triggering retry.')
                self._transition(State.RETRY)

        elif self.state == State.WAITING_TARGET:
            if self.tick_counter >= self.WAITING_TARGET_TIMEOUT_S:
                self.get_logger().warn('WAITING_TARGET timeout. Triggering retry.')
                self._transition(State.RETRY)

        elif self.state == State.NAVIGATING:
            if self.tick_counter >= self.NAV_TIMEOUT_S:
                self.get_logger().warn('NAVIGATING timeout. Triggering retry.')
                self._transition(State.RETRY)

    def _transition(self, new_state: State):
        self.get_logger().info(f'State transition: {self.state.name} --> {new_state.name}')
        self.state = new_state
        self.tick_counter = 0

        if new_state == State.RECON:
            self._enter_recon()
        elif new_state == State.MAP_READY:
            self._enter_map_ready()
        elif new_state == State.WAITING_TARGET:
            self._enter_waiting_target()
        elif new_state == State.NAVIGATING:
            self._enter_navigating()
        elif new_state == State.DONE:
            self._enter_done()
        elif new_state == State.RETRY:
            self._enter_retry()

    def _enter_recon(self):
        self.get_logger().info('Dispatching UAV for aerial reconnaissance...')
        self.pub_uav.publish(Bool(data=True))

    def _enter_map_ready(self):
        if self.map_result is None:
            self.get_logger().error('map_result is None. Cannot inject map.')
            self._transition(State.RETRY)
            return

        grid = self.map_result.map
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = 'map'
        self.pub_map.publish(grid)
        self.get_logger().info(
            f'Map injected: {grid.info.width}x{grid.info.height} cells, '
            f'resolution={grid.info.resolution:.3f} m/cell')

        if 'navigate_to_target' in self.task_names:
            self._transition(State.WAITING_TARGET)
        else:
            self.get_logger().info('Recon-only mission. No navigation required.')
            self._transition(State.DONE)

    def _enter_waiting_target(self):
        self.get_logger().info(
            'Map injected. Waiting for camera to detect target...')

    def _enter_navigating(self):
        if self.target_pose is None:
            self.get_logger().error('target_pose is None. Cannot navigate.')
            self._transition(State.RETRY)
            return

        target = self.target_pose
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = 'map'
        self.pub_goal.publish(target)
        self.get_logger().info(
            f'Goal pose sent: x={target.pose.position.x:.2f}, '
            f'y={target.pose.position.y:.2f}')

    def _enter_done(self):
        goal_zone = self.task_cmd.goal_zone if self.task_cmd else 'unknown'
        self.get_logger().info(
            f'Mission complete! goal_zone={goal_zone}. System returning to IDLE.')
        self.task_cmd = None
        self.task_names = []
        self.map_result = None
        self.target_pose = None
        self.retry_count = 0
        self.state = State.IDLE

    def _enter_retry(self):
        self.retry_count += 1
        if self.retry_count > self.MAX_RETRIES:
            self.get_logger().error(
                f'Max retries ({self.MAX_RETRIES}) exceeded. Mission failed. Returning to IDLE.')
            intent = self.task_cmd.intent if self.task_cmd else 'unknown'
            self.pub_feedback.publish(String(data=(
                f'Mission failed after {self.MAX_RETRIES} retries. '
                f'UAV or camera unresponsive. Original intent: {intent}.'
            )))
            self.task_cmd = None
            self.task_names = []
            self.map_result = None
            self.target_pose = None
            self.retry_count = 0
            self.state = State.IDLE
        else:
            self.get_logger().warn(
                f'Retry {self.retry_count}/{self.MAX_RETRIES}. Re-dispatching UAV...')
            self.map_result = None
            self.target_pose = None
            self._transition(State.RECON)


def main(args=None):
    rclpy.init(args=args)
    node = SchedulerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
