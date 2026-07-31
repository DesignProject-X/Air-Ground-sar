#!/usr/bin/env python3
"""
CrazyFlie Mission Bridge Node
CrazyFlie 任务桥接节点

Bridges scheduler dispatch (start point + search direction) to real flight
control: Takeoff -> GoTo(start) -> start wall-following exploration ->
on camera target detection: stop exploration -> notify_setpoints_stop ->
GoTo(target) -> linger to let the mapper finish covering the surroundings
-> report recon complete via MapResult.
将 scheduler 的派发(起点+探索方向)桥接到实机飞控:起飞 -> 飞到起点 ->
开始沿墙探索 -> 相机发现目标后停止探索 -> 清除速度设定点 -> 飞向目标 ->
悬停让建图补全周边 -> 上报侦察完成(MapResult)。

Usage / 用法:
    Terminal A: ros2 launch crazyflie_ros2_multiranger_bringup wall_follower_mapper_real.launch.py
    (cf_mission_node is launched as part of that file)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Empty
from std_srvs.srv import Trigger, SetBool
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from crazyflie_interfaces.msg import LogDataGeneric
from crazyflie_interfaces.srv import Takeoff, GoTo, NotifySetpointsStop
from builtin_interfaces.msg import Duration
from custom_msgs.msg import UavDispatch, MapResult
from custom_msgs.srv import StartWallFollowing


HOVER_HEIGHT     = 0.3   # Hover/search height in meters / 悬停与探索高度(米)
                         # Kept below the maze wall height (~0.4m) so the
                         # multiranger side sensors stay pointed at the walls
                         # instead of flying over them.
                         # 保持在迷宫墙高(约0.4米)以下,确保multiranger侧向传感器能
                         # 探测到墙,而不是飞到墙顶上方。
TAKEOFF_DURATION = 1.0  # 0.3m/1.0s = 0.3 m/s - matches the climb rate confirmed
                        # safe via manual Takeoff service testing in this
                        # project's small physical maze (see vel_mux.py's own
                        # TAKEOFF_DURATION for the same fix - this is a
                        # separate takeoff call, made directly by this node
                        # in _on_dispatch before wall-following/vel_mux even
                        # engage, so it needed its own fix).
                        # 0.3m/1.0秒 = 0.3m/s——匹配这个项目实体小迷宫里,通过
                        # 手动调用Takeoff service测试确认过的安全爬升速率(见
                        # vel_mux.py里同样的修复——这是这个节点在_on_dispatch里
                        # 自己独立发的一次起飞调用,发生在wall_following/vel_mux
                        # 介入之前,所以需要单独修一次)。
GOTO_DURATION    = 3.0   # Time to reach a GoTo goal / 飞向目标点用时(秒)
# Sitting on the ground puts the flow deck's PMW3901 at ~0.02m (measured via
# range.zrange), well under its ~0.08m minimum working distance, so it reports
# noise rather than real optical flow and the EKF integrates that into a
# position that drifts while the drone has not moved at all. Whether it drifts
# visibly depends on what happens to be under the lens - texture gives
# coherent-but-wrong flow that integrates steadily, a plain dark surface gives
# almost nothing - which is why the drift comes and goes between runs.
# Nothing in this project reset the estimator, so whatever drift accumulated
# before takeoff was carried into the flight as its starting position.
# 趴在地上时,flow deck 上的 PMW3901 距地约 0.02 米(由 range.zrange 实测),
# 远低于它约 0.08 米的最小工作距离,于是输出的是噪声而不是真实光流,EKF 把它
# 积分成位置——无人机根本没动,位置却一直在漂。漂得明不明显取决于镜头正下方
# 恰好是什么:有纹理会给出方向一致但错误的光流,积分成稳定漂移;单调暗色表面
# 则几乎没有输出——这就是为什么这个现象时有时无。
# 而这个项目里从来没有重置过估计器,所以起飞前积累的漂移会作为起飞时的初始
# 位置一路带进飞行里。
ESTIMATOR_RESET_PULSE_SECONDS = 0.3   # hold resetEstimation=1 this long
ESTIMATOR_RESET_SETTLE_SECONDS = 1.0  # let the EKF re-converge before takeoff
LINGER_SECONDS   = 5.0   # Extra hover time at target so the mapper covers
                         # surroundings before recon is reported complete
                         # 到达目标后额外悬停时间,让建图补全周边(秒)

# Position-based mapping-complete checkpoint, as an alternative to relying
# purely on a fixed timer: this maze is walked start-to-end (right corridor
# takeoff -> left corridor), so arriving at the left corridor means the whole
# maze has been covered and the search can stop, even if the camera hasn't
# found a target yet. Coordinates are in the map/odom frame established at
# takeoff (x = the drone's initial heading, not its current one - see
# _on_dispatch/_on_odom). MAPPING_MIN_SEARCH_SECONDS guards against
# triggering immediately if the takeoff point ever ends up within the
# checkpoint box - the maze layout already puts start/end far enough
# apart that this is mostly a defensive margin, not an expected trigger path.
# 基于位置的建图完成检查点,作为纯计时之外的判据:这个迷宫是从右侧走廊起飞
# 走到左侧走廊结束的,到达左侧走廊就说明整个迷宫都探索完了,就算相机还没
# 发现目标也可以停止搜索。坐标是起飞那一刻建立的map/odom坐标系(x轴方向是
# 起飞时的机头朝向,不是当前朝向——见_on_dispatch/_on_odom)。
# MAPPING_MIN_SEARCH_SECONDS是防止起飞点万一恰好落在检查点范围内导致
# 立刻触发的保护——按迷宫布局起点和终点本来就隔得够远,这个更多是防御性
# 余量,不是预期会触发的路径。
# A circular checkpoint (tried at both 0.15m and 0.30m radius, the latter
# paired with a front_range wall-facing condition to avoid matching the
# wrong corner) proved fragile/imprecise in practice. This box is a directly
# measured real-world checkpoint location instead - tight enough on its own
# to identify the correct spot without needing a second sensor-based
# condition.
# 圆形检查点(试过0.15m和0.30m两种半径,后者还配了front_range朝墙判据来
# 避免对错墙角)实测下来比较脆弱/不精确。这个矩形范围是直接实测出来的真实
# 检查点位置——本身就够精确,不需要再配第二个基于传感器的条件。
MAPPING_CHECKPOINT_X_MIN = 0.52
MAPPING_CHECKPOINT_X_MAX = 0.66
MAPPING_CHECKPOINT_Y_MIN = 0.89
MAPPING_CHECKPOINT_Y_MAX = 1.04
MAPPING_MIN_SEARCH_SECONDS = 20.0
LAND_SETTLE_SECONDS = 2.0  # Wait for stop_wall_following's landing to settle
                           # before reporting recon complete / 等待停止沿墙
                           # 飞行触发的降落稳定下来,再上报侦察完成(秒)

# Safety fallback: stop searching after this long even if the checkpoint was
# never reached (stuck in a corner, drifted off course, etc.) and no camera
# target showed up either - without this, a failure to reach the checkpoint
# means the drone searches forever, same gap as relying on time alone used to
# have.
# 安全兜底:就算一直没到检查点(比如卡在某个角落、飞偏了),而且相机也一直
# 没发现目标,搜索时间超过这个上限也会停止——没有这个兜底的话,一旦到不了
# 检查点,无人机就会一直搜索下去,跟纯靠时间停止那种方案原来的缺口是一样的。
MAPPING_SEARCH_TIMEOUT_SECONDS = 90.0  # 1min30s

# Mirror the vars lists of the matching log blocks in
# crazyflie_real_crazyswarm2.yaml - only used to sanity-check the arriving
# value count, so a yaml edit that forgets this side gets a warning instead
# of silently unpacking the wrong fields (see _on_height_debug/_on_thrust_debug).
# 跟crazyflie_real_crazyswarm2.yaml里对应日志块的vars列表保持一致——仅用于
# 核对收到的数值个数,这样万一改了yaml却忘了改这边,会得到一条警告,而不是
# 悄悄把字段解错(见_on_height_debug/_on_thrust_debug)。
HEIGHT_DEBUG_VARS = (
    'range.zrange', 'stateEstimate.z', 'stabilizer.roll', 'stabilizer.pitch',
)
THRUST_DEBUG_VARS = (
    'pm.vbat', 'motor.m1', 'motor.m2', 'motor.m3', 'motor.m4',
)
# Crazyflie brushed-motor PWM is a 16-bit value, so this is the ceiling a
# motor saturates against - the whole point of logging all four (see
# _on_thrust_debug).
# Crazyflie有刷电机的PWM是16位值,所以这就是电机饱和时顶到的上限——记录
# 四个电机就是为了看这个(见_on_thrust_debug)。
MOTOR_PWM_MAX = 65535


def make_duration(seconds: float) -> Duration:
    """Convert float seconds to ROS Duration message / 将秒数转为ROS Duration消息"""
    d = Duration()
    d.sec = int(seconds)
    d.nanosec = int((seconds - int(seconds)) * 1e9)
    return d


# An isolated single-cell "obstacle" is almost certainly sensor noise, not a
# real wall - simple_mapper_multiranger.py marks a cell occupied from a
# single ray endpoint, and a real wall gets hit by many ticks as the drone
# flies past it, so it shows up as a line of adjacent occupied cells, not one
# cell alone. Confirmed on a real drone-built map: exactly one point (sitting
# in open corridor space, blocking the only path between the maze's two
# halves) got flagged and cleared, nothing else. Checking a radius wider than
# the immediate 8 neighbors is deliberate - a real wall can still have gaps
# from the multiranger's sparse 4-ray coverage, and a too-tight check risks
# clearing part of a real (just sparsely-sampled) wall instead of only noise.
# 孤立的单格"障碍物"几乎肯定是传感器噪声,不是真实的墙——
# simple_mapper_multiranger.py是靠单次射线落点标记一个格子为占用的,真实的墙
# 会在无人机飞过的过程中被连续多次命中,画出来是一条连续相邻的占用格子线,
# 不会是单独一个格子。在真实无人机建的图上验证过:只有一个点(飘在开阔走廊
# 空间里、正好挡住迷宫两侧唯一通路)被标记并清除,没有动到别的地方。检查半径
# 特意选得比紧邻的8格更宽——真实的墙因为multiranger只有4条稀疏射线,本来就
# 可能有采样缝隙,查得太严格反而可能把稀疏但真实的墙段当噪声清掉。
MAP_DENOISE_RADIUS = 2
MAP_OCCUPIED_THRESHOLD = 65


def _denoise_map_data(data, width, height):
    cleaned = list(data)
    removed = 0
    for y in range(height):
        for x in range(width):
            idx = y * width + x
            if data[idx] < MAP_OCCUPIED_THRESHOLD:
                continue
            has_neighbor = False
            for dy in range(-MAP_DENOISE_RADIUS, MAP_DENOISE_RADIUS + 1):
                for dx in range(-MAP_DENOISE_RADIUS, MAP_DENOISE_RADIUS + 1):
                    if dx == 0 and dy == 0:
                        continue
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < width and 0 <= ny < height:
                        if data[ny * width + nx] >= MAP_OCCUPIED_THRESHOLD:
                            has_neighbor = True
                            break
                if has_neighbor:
                    break
            if not has_neighbor:
                cleaned[idx] = 0
                removed += 1
    return cleaned, removed


class CfMissionNode(Node):

    def __init__(self):
        super().__init__('cf_mission_node')

        self.declare_parameter('robot_prefix', 'crazyflie_real')
        robot_prefix = self.get_parameter('robot_prefix').value
        self.prefix = '/' + robot_prefix.lstrip('/')
        # Bare name, no leading slash: crazyflie_server keys its firmware-param
        # ROS parameters off the drone's name as written in crazyflies.yaml
        # ("<cf>.params.<group>.<name>"), not off a topic path - see
        # _reset_estimator.
        # 不带前导斜杠的裸名字:crazyflie_server 给固件参数用的 ROS 参数名,是以
        # crazyflies.yaml 里写的机器名为前缀的("<机器名>.params.<组>.<参数>"),
        # 不是话题路径——见 _reset_estimator。
        self.robot_name = robot_prefix.lstrip('/')

        # Service clients / 服务客户端
        self.takeoff_client   = self.create_client(Takeoff, self.prefix + '/takeoff')
        self.goto_client      = self.create_client(GoTo, self.prefix + '/go_to')
        self.notify_stop_client = self.create_client(
            NotifySetpointsStop, self.prefix + '/notify_setpoints_stop')
        self.start_wf_client  = self.create_client(
            StartWallFollowing, self.prefix + '/start_wall_following')
        self.stop_wf_client   = self.create_client(Trigger, self.prefix + '/stop_wall_following')
        self.set_mapping_active_client = self.create_client(
            SetBool, self.prefix + '/set_mapping_active')
        # Absolute name: crazyflie_server is launched into the root namespace
        # (see wall_follower_mapper_real.launch.py), while this node is not
        # necessarily - a relative name would resolve into the wrong namespace.
        # 绝对名称:crazyflie_server 是启动在根命名空间下的
        # (见 wall_follower_mapper_real.launch.py),而这个节点不一定是——用
        # 相对名称会解析到错误的命名空间。
        self.set_params_client = self.create_client(
            SetParameters, '/crazyflie_server/set_parameters')

        self.get_logger().info('Waiting for flight services... / 等待飞控服务就绪...')
        for client in (self.takeoff_client, self.goto_client, self.notify_stop_client,
                       self.start_wf_client, self.stop_wf_client,
                       self.set_mapping_active_client, self.set_params_client):
            client.wait_for_service(timeout_sec=5.0)
        self.get_logger().info('All services ready. / 所有服务就绪')

        # Latest map, cached for reporting recon complete / 缓存最新地图，用于上报侦察完成
        self.latest_map = None
        self.map_sub = self.create_subscription(
            OccupancyGrid, self.prefix + '/map', self._on_map,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       history=HistoryPolicy.KEEP_LAST))

        self.dispatch_sub = self.create_subscription(
            UavDispatch, '/uav/dispatch', self._on_dispatch, 10)
        self.target_sub = self.create_subscription(
            PoseStamped, '/camera/target_pose', self._on_target, 10)
        # The real camera is decoupled from the drone (it isn't onboard - see
        # coordinate_bridge_node's base_frame_id) and can detect the target
        # at any time regardless of how much of the maze the drone has
        # actually explored, so it isn't a valid "search complete" signal for
        # the drone - the scheduler already tracks it independently via its
        # own /camera/target_pose subscription for the ground robot's
        # eventual navigation. This separate topic lets a human manually
        # simulate "search complete" for the drone during testing, without
        # touching the real target topic at all.
        # 真相机跟无人机是解耦的(不在机上——见coordinate_bridge_node的
        # base_frame_id),不管无人机探索到哪了,相机随时可能检测到目标,
        # 所以拿它当无人机的"搜索完成"信号并不成立——调度器本来就有自己
        # 独立订阅的/camera/target_pose,用来给地面机器人最终导航用。这个
        # 单独的话题让测试时可以人为模拟"无人机搜索完成了",完全不用碰真实
        # 目标那个话题。
        self.debug_stop_search_sub = self.create_subscription(
            Empty, self.prefix + '/debug_stop_search', self._on_debug_stop_search, 10)
        self.odom_sub = self.create_subscription(
            Odometry, self.prefix + '/odom', self._on_odom, 10)
        # Diagnostic only - see _on_height_debug. The height_debug log block
        # is declared in crazyflie_real_crazyswarm2.yaml and published by
        # crazyflie_server as a plain topic, so without a subscriber its data
        # simply evaporates (confirmed live: a whole flight reproduced the
        # fault with the block correctly set up, and not one sample was kept
        # because nothing was recording). Subscribing here writes it into
        # this node's own ROS log every flight - same directory and clock as
        # the wall_following state log, so the two can be lined up directly -
        # instead of depending on remembering to start a rosbag by hand.
        # 仅用于诊断——见_on_height_debug。height_debug日志块在
        # crazyflie_real_crazyswarm2.yaml里声明,由crazyflie_server作为普通
        # 话题发布,所以没有订阅者的话数据就直接蒸发了(实测确认过:有一轮
        # 飞行成功复现了故障、日志块也配置正确,但因为没有任何东西在录,
        # 一个采样都没留下)。在这里订阅,可以让数据每次飞行都自动写进这个
        # 节点自己的ROS日志——跟wall_following的状态日志在同一个目录、同一个
        # 时钟,两边可以直接对齐——而不是依赖人记得手动起一次rosbag。
        self.height_debug_sub = self.create_subscription(
            LogDataGeneric, self.prefix + '/height_debug', self._on_height_debug, 10)
        self.thrust_debug_sub = self.create_subscription(
            LogDataGeneric, self.prefix + '/thrust_debug', self._on_thrust_debug, 10)
        self.map_result_pub = self.create_publisher(MapResult, '/uav/map_result', 10)

        # Mission state / 任务状态
        self._searching = False
        self._active_timer = None
        self._start_position = None
        self._start_yaw = 0.0
        self._direction = 'right'
        self._target_pose = None
        self._search_start_time = None
        self._search_timeout_timer = None

        self.get_logger().info('UAV mission node ready. Waiting for dispatch... / 就绪，等待派发...')

    def _cancel_active_timer(self):
        if self._active_timer is not None:
            self._active_timer.cancel()
            self._active_timer = None

    def _cancel_search_timeout(self):
        if self._search_timeout_timer is not None:
            self._search_timeout_timer.cancel()
            self._search_timeout_timer = None

    def _stop_search_and_report(self, reason: str):
        self._searching = False
        self._cancel_search_timeout()
        self.get_logger().info(reason)
        # Freeze the map before landing so the descent's brief attitude/
        # position disturbance doesn't get drawn into it - see
        # set_mapping_active in simple_mapper_multiranger.py.
        # 降落前先冻结地图,避免下降过程中短暂的姿态/位置扰动被画进去——见
        # simple_mapper_multiranger.py里的set_mapping_active。
        self.set_mapping_active_client.call_async(SetBool.Request(data=False))
        self.stop_wf_client.call_async(Trigger.Request())

        self._cancel_active_timer()
        self._active_timer = self.create_timer(LAND_SETTLE_SECONDS, self._report_recon_complete)

    def _on_map(self, msg: OccupancyGrid):
        self.latest_map = msg

    def _on_dispatch(self, msg: UavDispatch):
        self.get_logger().info(
            f'Dispatch received: start=({msg.start_position.x:.2f}, {msg.start_position.y:.2f}), '
            f'direction={msg.direction}')
        self._start_position = msg.start_position
        self._start_yaw = msg.start_yaw
        self._direction = msg.direction

        # Reset the estimator before commanding takeoff, then let it settle -
        # see _reset_estimator. Takeoff itself moves into _takeoff so the two
        # stay in order over the radio.
        # 起飞指令之前先重置估计器,再留一点稳定时间——见_reset_estimator。
        # 起飞本身挪到_takeoff里,好让这两件事在无线链路上保持先后顺序。
        self._cancel_active_timer()
        self._reset_estimator(value=1)
        self._active_timer = self.create_timer(
            ESTIMATOR_RESET_PULSE_SECONDS, self._end_estimator_reset)

    def _reset_estimator(self, value: int):
        # kalman.resetEstimation zeroes the EKF's position/velocity estimate.
        # It is a firmware parameter, and crazyflie_server exposes those as its
        # own ROS parameters named "<cf>.params.<group>.<name>" - setting one
        # makes its _parameters_callback forward the write to the drone (see
        # crazyflie_server.py). Hence SetParameters rather than a service of
        # our own.
        # Held at 1 for a moment and then released to 0, because the firmware
        # acts on the transition; setting both back to back over the radio can
        # be coalesced fast enough that the reset never takes.
        # kalman.resetEstimation 会把 EKF 的位置/速度估计清零。它是固件参数,
        # 而 crazyflie_server 把固件参数暴露成它自己的 ROS 参数,名字形如
        # "<机器名>.params.<组>.<参数>"——设置它,服务端的 _parameters_callback
        # 就会把这次写入转发给无人机(见 crazyflie_server.py)。所以这里用
        # SetParameters,而不是我们自己的服务。
        # 先置1保持一小会儿再置0,是因为固件是对这个跳变做出反应的;两次写入
        # 在无线链路上挨得太近有可能被合并掉,重置就不会真正发生。
        param = Parameter(
            name=f'{self.robot_name}.params.kalman.resetEstimation',
            value=ParameterValue(type=ParameterType.PARAMETER_INTEGER,
                                 integer_value=value))
        if not self.set_params_client.service_is_ready():
            self.get_logger().warn(
                'crazyflie_server/set_parameters not available - skipping estimator '
                'reset. Position drift accumulated on the ground will carry into '
                'this flight.')
            return
        self.set_params_client.call_async(SetParameters.Request(parameters=[param]))
        self.get_logger().info(f'kalman.resetEstimation set to {value}')

    def _end_estimator_reset(self):
        self._cancel_active_timer()
        self._reset_estimator(value=0)
        self._active_timer = self.create_timer(
            ESTIMATOR_RESET_SETTLE_SECONDS, self._takeoff)

    def _takeoff(self):
        self._cancel_active_timer()
        req = Takeoff.Request()
        req.group_mask = 0
        req.height = HOVER_HEIGHT
        req.duration = make_duration(TAKEOFF_DURATION)
        self.takeoff_client.call_async(req)
        self.get_logger().info(f'Takeoff command sent: height={HOVER_HEIGHT}m')

        # TEMP: skipping _goto_start and going straight to _begin_search after
        # takeoff - to restore, point this back at self._goto_start.
        # 临时:跳过_goto_start,起飞后直接进入_begin_search——要恢复的话,把
        # 这里改回指向self._goto_start即可。
        self._active_timer = self.create_timer(TAKEOFF_DURATION + 1.0, self._begin_search)

    def _goto_start(self):
        self._cancel_active_timer()
        req = GoTo.Request()
        req.group_mask = 0
        req.relative = False
        # start_position.z is always 0.0 (it's a ground-plan (x, y) point -
        # the scheduler's zone_cfg only has start_x/start_y/start_yaw, no
        # z at all) - flying straight to it as given would command a
        # descent-to-the-floor GoTo right after takeoff, since the drone is
        # already hovering at HOVER_HEIGHT by this point. Keep x/y from the
        # dispatch, override z to the hover height instead.
        # start_position.z恒为0.0(它本来就是地面平面上的(x,y)点——调度器的
        # zone_cfg只有start_x/start_y/start_yaw,根本没有z这个概念),如果
        # 原样飞过去,会在起飞之后立刻命令一次"降到地板"的GoTo,因为这时候
        # 无人机已经悬停在HOVER_HEIGHT了。这里x/y还是用派发过来的值,z改成
        # 悬停高度。
        req.goal = Point(
            x=self._start_position.x,
            y=self._start_position.y,
            z=HOVER_HEIGHT,
        )
        req.yaw = self._start_yaw
        req.duration = make_duration(GOTO_DURATION)
        self.goto_client.call_async(req)
        self.get_logger().info(
            f'GoTo start sent: ({self._start_position.x:.2f}, {self._start_position.y:.2f}, '
            f'{HOVER_HEIGHT:.2f})')

        self._active_timer = self.create_timer(GOTO_DURATION + 1.0, self._begin_search)

    def _begin_search(self):
        self._cancel_active_timer()
        self.set_mapping_active_client.call_async(SetBool.Request(data=True))
        req = StartWallFollowing.Request()
        req.direction = self._direction
        self.start_wf_client.call_async(req)
        self._searching = True
        self._search_start_time = self.get_clock().now()
        self._cancel_search_timeout()
        self._search_timeout_timer = self.create_timer(
            MAPPING_SEARCH_TIMEOUT_SECONDS, self._on_search_timeout)
        self.get_logger().info(f'Wall-following search started, direction={self._direction}')

    def _on_search_timeout(self):
        self._cancel_search_timeout()
        if not self._searching:
            return
        self._stop_search_and_report(
            f'Search timeout ({MAPPING_SEARCH_TIMEOUT_SECONDS:.0f}s) reached without reaching '
            f'the mapping checkpoint or a camera target - stopping as a safety fallback.'
        )

    def _log_data_ok(self, msg: LogDataGeneric, expected, name) -> bool:
        if len(msg.values) >= len(expected):
            return True
        self.get_logger().warn(
            f'{name}: expected {len(expected)} values, got {len(msg.values)} - '
            f'is the log block still in sync with crazyflie_real_crazyswarm2.yaml?',
            throttle_duration_sec=5.0)
        return False

    def _on_height_debug(self, msg: LogDataGeneric):
        # values arrive in exactly the order the vars are listed for the
        # height_debug block in crazyflie_real_crazyswarm2.yaml - keep the
        # two in sync if either is ever edited.
        # values的顺序跟crazyflie_real_crazyswarm2.yaml里height_debug块的
        # vars顺序完全一致——以后改动其中任何一边,记得同步另一边。
        if not self._log_data_ok(msg, HEIGHT_DEBUG_VARS, 'height_debug'):
            return
        zrange_mm, est_z, roll, pitch = msg.values[:4]
        # zrange is the raw downward laser in mm; est_z is the EKF's own
        # height in m. Reporting the gap between them directly is the whole
        # point of this block - a large, growing gap means the estimator has
        # left the measurement behind (estimator divergence), while the two
        # moving together means the drone really is going where the sensor
        # says it is.
        # zrange是向下激光的原始读数(毫米),est_z是EKF自己给出的高度(米)。
        # 直接把两者的差报出来正是这个日志块的意义——差值大且还在扩大,说明
        # 估计器已经甩开了实测值(估计器发散);两者一起动,说明无人机确实在
        # 往传感器所说的方向走。
        zrange_m = zrange_mm / 1000.0
        self.get_logger().info(
            f'HEIGHT_DEBUG zrange={zrange_m:.3f}m est_z={est_z:.3f}m '
            f'gap={est_z - zrange_m:+.3f}m roll={roll:+.1f} pitch={pitch:+.1f}')

    def _on_thrust_debug(self, msg: LogDataGeneric):
        if not self._log_data_ok(msg, THRUST_DEBUG_VARS, 'thrust_debug'):
            return
        vbat = msg.values[0]
        motors = msg.values[1:5]
        # Report the headroom left on the most-loaded motor, since that is
        # the one that runs out first and the number this block exists to
        # answer: near 0% while the drone is descending means the mixer had
        # nothing left to hold altitude with (thrust saturation), whereas
        # comfortable headroom during a descent rules thrust out entirely.
        # The spread across the four is logged alongside it because yaw is
        # produced by splitting the two rotation pairs apart - a large spread
        # is what pushes one pair into the ceiling in the first place.
        # 报告负载最高的那个电机还剩多少余量,因为它是最先耗尽的一个,也正是
        # 这个日志块要回答的数字:下降过程中余量接近0,说明混控已经没有余力
        # 维持高度了(推力饱和);而下降时余量还很充足,就能彻底排除推力这条
        # 线。旁边一起记录四个电机的极差,是因为偏航正是靠把两组反向桨拉开
        # 差距来产生的——极差大,才是把某一组顶到上限的直接原因。
        peak = max(motors)
        headroom_pct = 100.0 * (MOTOR_PWM_MAX - peak) / MOTOR_PWM_MAX
        self.get_logger().info(
            f'THRUST_DEBUG vbat={vbat:.2f}V '
            f'm=({motors[0]:.0f},{motors[1]:.0f},{motors[2]:.0f},{motors[3]:.0f}) '
            f'peak={peak:.0f} headroom={headroom_pct:.1f}% '
            f'spread={peak - min(motors):.0f}')

    def _on_odom(self, msg: Odometry):
        if not self._searching or self._search_start_time is None:
            return

        elapsed_s = (self.get_clock().now() - self._search_start_time).nanoseconds / 1e9
        if elapsed_s < MAPPING_MIN_SEARCH_SECONDS:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        in_checkpoint_box = (
            MAPPING_CHECKPOINT_X_MIN <= x <= MAPPING_CHECKPOINT_X_MAX
            and MAPPING_CHECKPOINT_Y_MIN <= y <= MAPPING_CHECKPOINT_Y_MAX
        )
        if not in_checkpoint_box:
            return

        self._stop_search_and_report(
            f'Mapping checkpoint reached at ({x:.2f}, {y:.2f}) after {elapsed_s:.1f}s of '
            f'searching - stopping without waiting for a camera target.'
        )

    def _on_target(self, msg: PoseStamped):
        # TEMP: no longer stops the search - the real camera is decoupled
        # from the drone and can report a target at any time regardless of
        # the drone's own exploration progress (see the subscription comment
        # above), so it's not a valid signal for the drone to act on here.
        # Just cache it in case _goto_target/flying-to-target is restored
        # later. Use debug_stop_search (see _on_debug_stop_search) to
        # manually end the search during testing instead.
        # 临时:不再触发停止搜索了——真相机跟无人机是解耦的,不管无人机自己
        # 探索到哪,随时都可能报出目标(见上面订阅那里的注释),所以这里不是
        # 一个无人机能拿来用的有效信号。先缓存起来,以防以后恢复
        # _goto_target/飞向目标这个行为用得上。测试时想手动结束搜索,改用
        # debug_stop_search(见_on_debug_stop_search)。
        self._target_pose = msg

    def _on_debug_stop_search(self, msg: Empty):
        if not self._searching:
            return
        self._stop_search_and_report(
            'debug_stop_search received - manually stopping search (testing only).'
        )

    def _goto_target(self):
        self._cancel_active_timer()
        req = GoTo.Request()
        req.group_mask = 0
        req.relative = False
        # Same issue _goto_start had: the camera reports the target's ground
        # position, not a flight altitude - flying straight to its z would
        # command a descent-to-the-floor GoTo. Keep x/y from the detection,
        # override z to the hover height so the drone flies over the target
        # instead of down into it.
        # 跟_goto_start是同一个问题:相机上报的是目标在地面上的位置,不是
        # 飞行高度——直接用它的z飞过去,会变成一次"降到地板"的GoTo。x/y还是
        # 用检测到的值,z改成悬停高度,让无人机飞到目标正上方,而不是直接
        # 冲向目标降落。
        req.goal = Point(
            x=self._target_pose.pose.position.x,
            y=self._target_pose.pose.position.y,
            z=HOVER_HEIGHT,
        )
        req.yaw = 0.0
        req.duration = make_duration(GOTO_DURATION)
        self.goto_client.call_async(req)
        self.get_logger().info(
            f'GoTo target sent: ({self._target_pose.pose.position.x:.2f}, '
            f'{self._target_pose.pose.position.y:.2f}, {HOVER_HEIGHT:.2f})')

        self._active_timer = self.create_timer(GOTO_DURATION + LINGER_SECONDS,
                                                self._report_recon_complete)

    def _report_recon_complete(self):
        self._cancel_active_timer()
        if self.latest_map is None:
            self.get_logger().error('No map received yet; cannot report recon complete.')
            return

        result = MapResult()
        result.map = self.latest_map
        cleaned_data, removed = _denoise_map_data(
            result.map.data, result.map.info.width, result.map.info.height)
        result.map.data = cleaned_data
        if removed:
            self.get_logger().info(
                f'Map denoise: cleared {removed} isolated occupied cell(s) before reporting.')
        result.confidence = 0.9
        result.frame_id = 'map'
        self.map_result_pub.publish(result)
        self.get_logger().info('Recon complete. MapResult published.')


def main(args=None):
    rclpy.init(args=args)
    node = CfMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
