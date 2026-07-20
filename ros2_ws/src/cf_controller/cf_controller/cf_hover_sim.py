#!/usr/bin/env python3
"""
CrazyFlie Simulation Hover Node
CrazyFlie 仿真悬停节点

Sequence: climb to HOVER_HEIGHT -> hold indefinitely -> on stop_hover call, land.
流程: 爬升到 HOVER_HEIGHT -> 一直悬停 -> 收到 stop_hover 服务调用后降落

Unlike cf_hover_real.py (which uses the crazyflie_interfaces Takeoff/Land
services), the simulation stack has no such services - control_services.py
only understands /cmd_vel.linear.z as a takeoff/land trigger, so this node
drives the mission by publishing Twist directly and watching /odom for
height feedback, the same interface cf_basic_control.py used.
和 cf_hover_real.py 不同(那边用 crazyflie_interfaces 的 Takeoff/Land 服务),
仿真这边没有这些服务 - control_services.py 只认 /cmd_vel.linear.z 作为起飞/
降落的触发信号,所以这个节点直接发布 Twist,并订阅 /odom 读高度反馈,
和 cf_basic_control.py 用的是同一套接口。

This node is meant to be the ONLY thing publishing /cmd_vel while it runs.
Do not run it at the same time as wall_following_multiranger - both publish
to the same /cmd_vel topic and control_services.py only ever looks at
whichever message arrived last, so two publishers racing on it corrupts
both the climb and the hover/land commands. The auto-handoff mode below is
the one sanctioned exception: it cancels its own /cmd_vel timer in the same
tick it calls start_wall_following, so there is no tick where both nodes are
publishing.
这个节点运行期间应该是唯一往 /cmd_vel 发消息的节点。不要和
wall_following_multiranger 同时跑 - 两者发到同一个 /cmd_vel 话题,
control_services.py 只看最后收到的那一条,两边抢着发会同时搞乱起飞爬升和
悬停/降落指令。下面的自动交接模式是唯一的例外——它在调用 start_wall_following
的同一个 tick 里就取消了自己的 /cmd_vel timer,不存在两边同时在发的那一刻。

Usage / 用法:
    Terminal A: ros2 launch crazyflie_ros2_multiranger_bringup wall_follower_mapper_simulation.launch.py
                (or any launch that brings up control_services under the same robot_prefix)
    Terminal B: ros2 run cf_controller cf_hover_sim
    Terminal C (when ready to land): ros2 service call /crazyflie/stop_hover std_srvs/srv/Trigger

Optional auto-handoff to wall_following / 可选:自动交接给巡墙节点
    Set the `auto_wallfollow_direction` parameter (e.g. 'left') and this node
    will, once it reaches hover height, call wall_following_multiranger's
    start_wall_following service itself and then release /cmd_vel - no
    manual pub/service-call needed. Leave it unset (default '') to keep the
    plain hover-until-stop_hover behavior above.
    设置 `auto_wallfollow_direction` 参数(比如 'left')后,爬升到悬停高度会
    自动调用 wall_following_multiranger 的 start_wall_following service,然后
    让出 /cmd_vel - 不需要再手动 pub/调 service。不设置(默认 '')就还是上面
    那种"悬停直到 stop_hover"的普通用法。
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from custom_msgs.srv import StartWallFollowing


# Flight parameters / 飞行参数
ROBOT_PREFIX  = '/crazyflie'  # Must match the robot_prefix used by control_services
                              # and the wall_following/mapper nodes in the sim launch files.
                              # 必须和仿真 launch 文件里 control_services、沿墙/建图节点用的
                              # robot_prefix 一致。
# NOTE: this must match the `hover_height` parameter given to control_services
# in crazyflie_simulation.launch.py (currently 0.25m) - that value, not this
# one, is what actually caps the climb. If they drift apart, this node keeps
# sending a positive-z command past the point control_services already
# switched to is_flying=True, and once flying, control_services no longer
# caps a positive z command - the drone just keeps climbing uncapped.
# 必须和 crazyflie_simulation.launch.py 里传给 control_services 的
# hover_height 参数(目前是0.25m)保持一致 - 真正限制爬升高度的是那个值,不是
# 这里的值。两边不一致的话,control_services 内部其实已经切到 is_flying=True
# 了,但这个节点还在发正的z指令 - 一旦进入飞行状态,control_services 不会再
# 限制正向z指令,无人机会一直不受控地往上爬。
HOVER_HEIGHT      = 0.25   # Hover height in meters / 悬停高度(米)
# IMPORTANT: control_services.py only flips its internal is_flying=True once
# current height is STRICTLY GREATER than hover_height - and until is_flying
# is True, it silently drops every non-height command it receives (see the
# `if self.is_flying:` gate in its timer_callback), including wall_following's
# horizontal velocity commands after handoff. So this margin must push the
# stop-climbing check PAST hover_height, never short of it - stopping early
# (e.g. HOVER_HEIGHT - margin) leaves the height plateaued just under the
# threshold forever, is_flying never becomes True, and the drone silently
# never responds to horizontal commands again (this was an actual bug here:
# confirmed by a real run where wall_following kept commanding vx=0.5 but the
# drone's logged position never moved at all).
# 重要:control_services.py 只有在实际高度严格大于 hover_height 时才会把内部的
# is_flying 置为 True——在此之前,它的 timer_callback 里 `if self.is_flying:`
# 这道门会让它默默丢弃收到的所有非高度类指令,包括交接后 wall_following 发的水平
# 速度指令。所以这个余量必须让停止爬升的判断点落在 hover_height **之上**,不能
# 在其之下——如果提前(比如 HOVER_HEIGHT - margin)就停止爬升,高度会永远卡在
# 阈值以下一点点,is_flying 永远不会变 True,飞机之后就再也不会响应任何水平方向
# 的指令了(这是一个真实发生过的 bug:实测过 wall_following 一直在发 vx=0.5,
# 但飞机记录的位置坐标完全没有变化)。
HEIGHT_MARGIN     = 0.02   # Stop climbing only once safely PAST hover_height,
                           # not before / 只有安全越过 hover_height 之后才停止爬升
CLIMB_VZ          = 0.1    # Commanded z while climbing - only the sign matters
                           # to control_services pre-takeoff, the actual climb
                           # rate is fixed internally at 0.1 m/s
                           # 爬升阶段发送的z - 起飞前 control_services 只看符号,
                           # 实际爬升速率由它内部固定为0.1m/s
LAND_VZ           = -0.1   # Commanded z while landing / 降落阶段发送的z
LAND_HEIGHT       = 0.1    # Below this height, control_services considers
                           # landing complete / 低于这个高度视为降落完成


class CfHoverSim(Node):

    def __init__(self):
        super().__init__('cf_hover_sim')

        self.declare_parameter('auto_wallfollow_direction', '')
        self.auto_wallfollow_direction = self.get_parameter(
            'auto_wallfollow_direction').value

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, ROBOT_PREFIX + '/odom', self.odom_callback, 10)
        self.stop_srv = self.create_service(
            Trigger, ROBOT_PREFIX + '/stop_hover', self.stop_hover_cb)

        self.wall_follow_client = None
        if self.auto_wallfollow_direction:
            self.wall_follow_client = self.create_client(
                StartWallFollowing, ROBOT_PREFIX + '/start_wall_following')

        self.current_height = 0.0

        # State machine / 状态机:
        #   plain hover:  'climb' -> 'hover' -> 'land' -> 'done'
        #   auto handoff: 'climb' -> 'handoff' -> 'done'
        self.state = 'climb'

        self.timer = self.create_timer(0.1, self.control_loop)

        if self.auto_wallfollow_direction:
            self.get_logger().info(
                f'CrazyFlie sim hover node started. Climbing to {HOVER_HEIGHT}m, '
                f'then handing off to wall_following (direction='
                f'{self.auto_wallfollow_direction}). '
                f'/ 仿真悬停节点启动,爬升到{HOVER_HEIGHT}m后自动交接给巡墙节点'
                f'(direction={self.auto_wallfollow_direction})')
        else:
            self.get_logger().info(
                f'CrazyFlie sim hover node started. Climbing to {HOVER_HEIGHT}m, '
                f'then holding indefinitely. / 仿真悬停节点启动,爬升到{HOVER_HEIGHT}m后一直悬停')
            self.get_logger().info(
                f'Call {ROBOT_PREFIX}/stop_hover (std_srvs/Trigger) to land. '
                f'/ 调用 {ROBOT_PREFIX}/stop_hover 服务(std_srvs/Trigger)触发降落')

    def odom_callback(self, msg: Odometry):
        self.current_height = msg.pose.pose.position.z

    def stop_hover_cb(self, request, response):
        if self.state == 'hover':
            self.get_logger().info('Stop requested, landing. / 收到停止指令,开始降落')
            self.state = 'land'
            response.success = True
        else:
            response.success = False
            response.message = f'Ignored - not currently hovering (state={self.state})'
        return response

    def publish_cmd(self, vz=0.0):
        msg = Twist()
        msg.linear.z = vz
        self.cmd_pub.publish(msg)

    def control_loop(self):

        if self.state == 'climb':
            self.publish_cmd(vz=CLIMB_VZ)
            self.get_logger().info(
                f'Climbing... height={self.current_height:.2f}m',
                throttle_duration_sec=1.0)
            if self.current_height >= HOVER_HEIGHT + HEIGHT_MARGIN:
                if self.auto_wallfollow_direction:
                    self.state = 'handoff'
                    self.get_logger().info(
                        f'Reached hover height ({self.current_height:.2f}m). '
                        f'Handing off to wall_following. / 到达悬停高度,交接给巡墙节点')
                else:
                    self.state = 'hover'
                    self.get_logger().info(
                        f'Reached hover height ({self.current_height:.2f}m). '
                        f'Holding until stop_hover is called. / 到达悬停高度,持续悬停中')

        elif self.state == 'handoff':
            # Keep actively holding height every tick, including while we
            # wait for wall_following_multiranger's service to come up -
            # otherwise control_services.py just keeps re-using our last
            # published command (still the positive climb vz), and the drone
            # keeps climbing uncapped for the whole wait.
            # 每个 tick 都要主动发悬停指令,包括等 service 就绪的这段时间——
            # 不然 control_services.py 会一直复用我们发的最后一条指令(还是正的
            # 爬升 vz),导致等待期间飞机一直失控往上爬。
            self.publish_cmd(vz=0.0)
            if not self.wall_follow_client.service_is_ready():
                self.get_logger().info(
                    'Waiting for start_wall_following service...',
                    throttle_duration_sec=1.0)
                return
            request = StartWallFollowing.Request()
            request.direction = self.auto_wallfollow_direction
            self.wall_follow_client.call_async(request).add_done_callback(
                self._on_wall_follow_started)
            # Cancel now, in the same tick as the call - the last command we
            # ever published was the vz=0.0 hold above, so there's no stale
            # positive-z command left for control_services to keep re-using
            # once wall_following_multiranger's own timer takes over.
            # 就在发起调用的这个 tick 里取消 timer——我们发布的最后一条指令就是
            # 上面那条 vz=0.0 的悬停指令,所以巡墙节点的 timer 接管之后,不会有
            # 残留的正向 z 指令被 control_services 继续复用。
            self.timer.cancel()
            self.state = 'done'
            self.get_logger().info('Handoff complete. / 交接完成')

        elif self.state == 'hover':
            self.publish_cmd(vz=0.0)
            self.get_logger().info(
                f'Hovering... height={self.current_height:.2f}m',
                throttle_duration_sec=2.0)

        elif self.state == 'land':
            self.publish_cmd(vz=LAND_VZ)
            self.get_logger().info(
                f'Landing... height={self.current_height:.2f}m',
                throttle_duration_sec=1.0)
            if self.current_height < LAND_HEIGHT:
                self.publish_cmd(vz=0.0)
                self.state = 'done'
                self.get_logger().info('Landing complete. / 降落完成')

        # 'done': nothing left to publish, node just idles / 什么都不用发了

    def _on_wall_follow_started(self, future):
        result = future.result()
        if result is not None and result.success:
            self.get_logger().info(
                'wall_following_multiranger confirmed start_wall_following. '
                '/ 巡墙节点确认已启动')
        else:
            self.get_logger().error(
                'start_wall_following call failed - drone is hovering but not '
                'wall-following. / start_wall_following 调用失败——飞机在悬停但没有巡墙')


def main(args=None):
    rclpy.init(args=args)
    node = CfHoverSim()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted, landing. / 收到中断,开始降落')
        node.state = 'land'
        # Give the landing loop a few seconds to run before shutdown
        # 中断后让降落循环再跑几秒钟
        end_time = node.get_clock().now().nanoseconds + int(5e9)
        while rclpy.ok() and node.get_clock().now().nanoseconds < end_time and node.state != 'done':
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
