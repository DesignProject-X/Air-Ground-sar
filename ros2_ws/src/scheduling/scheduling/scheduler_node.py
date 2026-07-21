import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from enum import Enum, auto

from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from custom_msgs.msg import TaskCommand, MapResult, UavDispatch
from custom_msgs.srv import SaveMap
from std_srvs.srv import Trigger


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
    # cf_mission_node_sim's mapping_duration_sec defaults to 120s (see
    # cf_controller/cf_mission_node_sim.py) plus a few seconds of climb/
    # handoff before it even starts the timer - 60s cut that off before the
    # sim UAV finished mapping. 140s leaves margin above the full 120s.
    # cf_mission_node_sim的mapping_duration_sec默认是120秒(见
    # cf_controller/cf_mission_node_sim.py),而且计时器开始前还有几秒爬升/
    # 交接的时间——60秒会在仿真无人机建图完成前就把它切断。140秒在120秒
    # 之上留了余量。
    RECON_TIMEOUT_S = 140
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
        self.zone_cfg: dict = {}

        # Tracks whatever map_server on the ground robot is currently
        # serving. /map is TRANSIENT_LOCAL, so if a map was already loaded
        # before this node even started, this subscription still picks it
        # up immediately - letting a new task command skip re-dispatching
        # the UAV entirely when a usable map already exists.
        # 记录小车那边map_server现在正在提供的地图。/map是TRANSIENT_LOCAL的,
        # 就算这个节点启动之前地图就已经加载好了,这个订阅也能立刻收到——这样
        # 新任务如果已经有能用的地图,就可以完全跳过重新派无人机建图这一步。
        self.current_map: OccupancyGrid | None = None

        self._load_zones()

        self.create_subscription(TaskCommand, '/scheduler/task_command',
                                 self._on_task_command, 10)
        self.create_subscription(MapResult, '/uav/map_result',
                                 self._on_map_result, 10)
        self.create_subscription(
            OccupancyGrid, '/map', self._on_current_map,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       history=HistoryPolicy.KEEP_LAST))
        self.create_subscription(PoseStamped, '/camera/target_pose',
                                 self._on_camera_target, 10)
        self.create_subscription(Bool, '/goal_reached',
                                 self._on_goal_reached, 10)
        self.create_subscription(String, '/nav_status',
                                 self._on_nav_status, 10)

        self.pub_uav = self.create_publisher(UavDispatch, '/uav/dispatch', 10)
        self.pub_goal = self.create_publisher(PoseStamped, '/ground/goal_pose', 10)
        self.pub_feedback = self.create_publisher(String, '/planner/feedback', 10)
        # TRANSIENT_LOCAL so a dashboard that connects after the state
        # machine has already moved on still gets the current state
        # immediately, instead of waiting for the next transition.
        # TRANSIENT_LOCAL是为了让状态机已经往前走了之后才连上的仪表盘,
        # 也能立刻拿到当前状态,不用等下一次状态切换。
        self.pub_state = self.create_publisher(
            String, '/scheduler/state',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       history=HistoryPolicy.KEEP_LAST))
        # Ground-robot side map_receiver_node saves this to its own local
        # disk and reloads map_server from there - no direct /map publish,
        # since map_server (already running on the ground robot) is itself
        # the sole legitimate publisher of that topic; publishing it again
        # here would just race with map_server's own copy.
        # 小车那边的map_receiver_node会把这个存到它自己本地磁盘,再让
        # map_server重新加载——这里不直接发布/map,因为map_server(已经在
        # 小车上跑着)本来就是这个话题唯一该有的发布者,这里再发一次只会跟
        # map_server自己那份互相打架。
        self.save_map_client = self.create_client(SaveMap, 'save_map')
        # Asks the ground robot "do you already have a map saved from a
        # previous run" - the decision of whether that's good enough to
        # skip aerial recon belongs here in the scheduler, not baked into
        # the robot's own launch-time configuration.
        # 问机器人"你有没有之前跑保存下来的地图"——用不用这份地图跳过UAV建图,
        # 这个决策该由调度器来做,而不是写死在机器人自己的启动配置里。
        self.load_existing_map_client = self.create_client(Trigger, 'load_existing_map')

        self.create_timer(1.0, self._tick)

        self._publish_state()
        self.get_logger().info('Scheduling Layer started. Waiting for task command...')

    def _publish_state(self):
        self.pub_state.publish(String(data=self.state.name))

    def _load_zones(self):
        self.declare_parameter('zone_names', ['default'])
        zone_names = self.get_parameter('zone_names').value
        self.zones = {}
        for name in zone_names:
            self.declare_parameter(f'{name}.start_x', 0.0)
            self.declare_parameter(f'{name}.start_y', 0.0)
            self.declare_parameter(f'{name}.start_yaw', 0.0)
            self.declare_parameter(f'{name}.direction', 'right')
            self.zones[name] = {
                'start_x': self.get_parameter(f'{name}.start_x').value,
                'start_y': self.get_parameter(f'{name}.start_y').value,
                'start_yaw': self.get_parameter(f'{name}.start_yaw').value,
                'direction': self.get_parameter(f'{name}.direction').value,
            }
        self.zones.setdefault('default', {
            'start_x': 0.0, 'start_y': 0.0, 'start_yaw': 0.0, 'direction': 'right',
        })

    def _lookup_zone(self, goal_zone: str) -> dict:
        key = (goal_zone or '').strip().lower().replace(' ', '_')
        if key in self.zones:
            return self.zones[key]
        self.get_logger().warn(f'Unknown goal_zone "{goal_zone}", falling back to default zone.')
        return self.zones['default']

    def _on_task_command(self, msg: TaskCommand):
        if self.state != State.IDLE:
            self.get_logger().warn(
                f'New command received but current state is {self.state.name}. Ignored.')
            return
        self.task_cmd = msg
        self.task_names = [t.task for t in msg.sequence]
        self.zone_cfg = self._lookup_zone(msg.goal_zone)
        self.retry_count = 0
        self.get_logger().info(
            f'Task command received: intent={msg.intent}, goal_zone={msg.goal_zone}, '
            f'sequence={self.task_names}')

        SUPPORTED = {'aerial_recon', 'map_injection', 'navigate_to_target'}
        for name in self.task_names:
            if name not in SUPPORTED:
                self.get_logger().warn(f'Task [{name}] is not yet supported by the scheduler.')

        if self.current_map is not None:
            self.get_logger().info(
                'Existing map already available on the ground robot - '
                'skipping aerial recon and map injection.')
            self.get_logger().info('State transition: IDLE --> MAP_READY (reusing existing map)')
            self.state = State.MAP_READY
            self.tick_counter = 0
            self._proceed_after_map_ready()
        else:
            self._try_reuse_saved_map()

    def _try_reuse_saved_map(self):
        if not self.load_existing_map_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                'load_existing_map service not available on ground robot. '
                'Dispatching aerial recon.')
            self._transition(State.RECON)
            return
        future = self.load_existing_map_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_reuse_saved_map_response)

    def _on_reuse_saved_map_response(self, future):
        result = future.result()
        if result is not None and result.success:
            self.get_logger().info(
                f'Ground robot already had a saved map: {result.message} '
                'Skipping aerial recon.')
            self.get_logger().info('State transition: IDLE --> MAP_READY (reusing saved map)')
            self.state = State.MAP_READY
            self.tick_counter = 0
            self._proceed_after_map_ready()
        else:
            msg = result.message if result is not None else 'service call failed'
            self.get_logger().info(f'No usable saved map on ground robot ({msg}). '
                                    'Dispatching aerial recon.')
            self._transition(State.RECON)

    def _on_current_map(self, msg: OccupancyGrid):
        self.current_map = msg

    def _on_map_result(self, msg: MapResult):
        if self.state != State.RECON:
            return
        self.map_result = msg
        self.get_logger().info(
            f'Map result received: confidence={msg.confidence:.2f}, frame={msg.frame_id}')
        self._transition(State.MAP_READY)

    def _on_camera_target(self, msg: PoseStamped):
        if self.state not in (State.RECON, State.WAITING_TARGET):
            return
        self.target_pose = msg
        self.get_logger().info(
            f'Camera target received: x={msg.pose.position.x:.2f}, '
            f'y={msg.pose.position.y:.2f}')
        if self.state == State.WAITING_TARGET:
            self._transition(State.NAVIGATING)
        else:
            self.get_logger().info(
                'Target detected during RECON. Will skip WAITING_TARGET once map is ready.')

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
        self._publish_state()

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
        dispatch = UavDispatch()
        dispatch.start_position.x = self.zone_cfg['start_x']
        dispatch.start_position.y = self.zone_cfg['start_y']
        dispatch.start_position.z = 0.0
        dispatch.start_yaw = self.zone_cfg['start_yaw']
        dispatch.direction = self.zone_cfg['direction']
        self.pub_uav.publish(dispatch)
        self.get_logger().info(
            f'Dispatching UAV for aerial reconnaissance: '
            f'start=({dispatch.start_position.x:.2f}, {dispatch.start_position.y:.2f}), '
            f'direction={dispatch.direction}')

    def _enter_map_ready(self):
        if self.map_result is None:
            self.get_logger().error('map_result is None. Cannot inject map.')
            self._transition(State.RETRY)
            return

        grid = self.map_result.map
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = 'map'

        if not self.save_map_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(
                'save_map service not available on ground robot. Cannot inject map.')
            self._transition(State.RETRY)
            return

        req = SaveMap.Request()
        req.map = grid
        future = self.save_map_client.call_async(req)
        future.add_done_callback(self._on_save_map_response)
        self.get_logger().info(
            f'Map sent to ground robot: {grid.info.width}x{grid.info.height} cells, '
            f'resolution={grid.info.resolution:.3f} m/cell. Waiting for save/load confirmation...')

    def _on_save_map_response(self, future):
        result = future.result()
        if result is None or not result.success:
            msg = result.message if result is not None else 'service call failed'
            self.get_logger().error(f'Ground robot failed to save/load map: {msg}')
            self._transition(State.RETRY)
            return

        self.get_logger().info(f'Ground robot confirmed map saved and loaded: {result.message}')
        self._proceed_after_map_ready()

    def _proceed_after_map_ready(self):
        if 'navigate_to_target' not in self.task_names:
            self.get_logger().info('Recon-only mission. No navigation required.')
            self._transition(State.DONE)
        elif self.target_pose is not None:
            self.get_logger().info('Target already detected during recon. Skipping WAITING_TARGET.')
            self._transition(State.NAVIGATING)
        else:
            self._transition(State.WAITING_TARGET)

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
        self._publish_state()

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
            self._publish_state()
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
