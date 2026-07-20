#!/usr/bin/env python3
"""
CrazyFlie 基础控制节点：起飞 → 悬停 3 秒 → 降落
------------------------------------------------------
用法（仿真）：
    1. 先跑 wall_follower_mapper_simulation.launch.py（让 control_services 节点上线）
    2. 新终端：source ~/cf_ws/install/setup.bash
    3. python3 cf_basic_control.py

话题说明：
    发布 → /cmd_vel (geometry_msgs/Twist)
        control_services 订阅这个话题，再转发给 /crazyflie/cmd_vel
    订阅 → /crazyflie/odom (nav_msgs/Odometry)
        读取无人机当前高度，判断起飞/降落状态

实机迁移说明：
    实机时把话题名 /cmd_vel 改成 crazyswarm2 对应的话题即可，
    控制逻辑完全不用改。
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time


class CfBasicControl(Node):

    def __init__(self):
        super().__init__('cf_basic_control')

        # 发布控制指令到 control_services 订阅的话题
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 订阅里程计，读取当前高度
        self.odom_sub = self.create_subscription(
            Odometry,
            '/crazyflie/odom',
            self.odom_callback,
            10
        )

        # 当前高度（从 odom 更新）
        self.current_height = 0.0

        # 目标悬停高度（和 control_services 的默认值一致）
        self.hover_height = 1

        # 状态标志
        self.is_flying = False
        self.hovering = False
        self.landing = False
        self.mission_done = False

        # 悬停开始时间（用于计时 3 秒）
        self.hover_start_time = None

        # 主控制定时器，每 0.1 秒执行一次
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('=== CrazyFlie 基础控制节点启动 ===')
        self.get_logger().info('流程：起飞 → 悬停 3 秒 → 降落')

    def odom_callback(self, msg: Odometry):
        """更新当前高度"""
        self.current_height = msg.pose.pose.position.z

    def publish_cmd(self, vx=0.0, vy=0.0, vz=0.0, az=0.0):
        """发布速度指令"""
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.linear.z = vz    # 正值=上升，负值=下降，0=维持高度
        msg.angular.z = az
        self.cmd_pub.publish(msg)

    def control_loop(self):
        """主控制逻辑，每 0.1 秒执行一次"""

        # 任务完成后停止
        if self.mission_done:
            return

        # ── 阶段 1：起飞 ──────────────────────────────────────────
        # 发 linear.z = 0.5（正值）触发 control_services 的起飞逻辑
        # 它会自动爬升到 0.5m 后切换为悬停
        if not self.is_flying and not self.hovering and not self.landing:
            self.get_logger().info(
                f'起飞中... 当前高度: {self.current_height:.2f}m',
                throttle_duration_sec=1.0   # 每秒只打印一次，避免刷屏
            )
            self.publish_cmd(vz=0.5)

            # 检测是否到达悬停高度
            if self.current_height >= self.hover_height - 0.05:
                self.is_flying = True
                self.hovering = True
                self.hover_start_time = self.get_clock().now()
                self.get_logger().info(
                    f'✅ 起飞完成！当前高度: {self.current_height:.2f}m，开始悬停 3 秒'
                )

        # ── 阶段 2：悬停 3 秒 ────────────────────────────────────
        # 发 linear.z = 0，control_services 会自动维持当前高度
        elif self.hovering and not self.landing:
            self.publish_cmd(vz=0.0)    # 发零速度，触发高度维持逻辑

            # 计算已悬停时间
            elapsed = (self.get_clock().now() - self.hover_start_time).nanoseconds / 1e9
            self.get_logger().info(
                f'悬停中... 高度: {self.current_height:.2f}m，已悬停: {elapsed:.1f}s / 3.0s',
                throttle_duration_sec=1.0
            )

            # 悬停满 3 秒后开始降落
            if elapsed >= 3.0:
                self.hovering = False
                self.landing = True
                self.get_logger().info('✅ 悬停 3 秒完成，开始降落')

        # ── 阶段 3：降落 ─────────────────────────────────────────
        # 发 linear.z = -0.3（负值）触发降落
        # control_services 检测到高度 < 0.1m 后自动停止
        elif self.landing:
            self.get_logger().info(
                f'降落中... 当前高度: {self.current_height:.2f}m',
                throttle_duration_sec=1.0
            )
            self.publish_cmd(vz=-0.3)   # 负值 = 下降

            # 检测是否降落完成
            if self.current_height < 0.1:
                self.publish_cmd(vz=0.0)    # 停止所有指令
                self.mission_done = True
                self.get_logger().info('✅ 降落完成！任务结束。')


def main(args=None):
    rclpy.init(args=args)
    node = CfBasicControl()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Ctrl+C 时发送零速度让无人机悬停
        node.get_logger().info('收到中断信号，发送悬停指令...')
        node.publish_cmd(vz=0.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()