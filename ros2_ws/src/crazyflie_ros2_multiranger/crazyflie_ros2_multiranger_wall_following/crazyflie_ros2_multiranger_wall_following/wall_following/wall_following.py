"""
file: wall_following.py

Class for the wall following demo

This is a python port of c-based app layer example from the Crazyflie-firmware
found here https://github.com/bitcraze/crazyflie-firmware/tree/master/examples/
demos/app_wall_following_demo

Author:   Kimberly McGuire (Bitcraze AB)
"""
import math
from enum import Enum


class WallFollowing():
    class StateWallFollowing(Enum):
        FORWARD = 1
        HOVER = 2
        TURN_TO_FIND_WALL = 3
        TURN_TO_ALIGN_TO_WALL = 4
        FORWARD_ALONG_WALL = 5
        ROTATE_AROUND_WALL = 6
        ROTATE_IN_CORNER = 7
        FIND_CORNER = 8

    class WallFollowingDirection(Enum):
        LEFT = 1
        RIGHT = -1

    def __init__(self, reference_distance_from_wall=0.5,
                 max_forward_speed=0.2,
                 max_turn_rate=0.5,
                 wall_following_direction=WallFollowingDirection.LEFT,
                 first_run=False,
                 prev_heading=0.0,
                 wall_angle=0.0,
                 around_corner_back_track=False,
                 state_start_time=0.0,
                 ranger_value_buffer=0.2,
                 angle_value_buffer=0.1,
                 range_lost_threshold=0.3,
                 in_corner_angle=0.8,
                 wait_for_measurement_seconds=1.0,
                 wall_too_close_distance=0.2,
                 wall_too_far_distance=0.4,
                 front_wall_detect_distance=None,
                 init_state=StateWallFollowing.FORWARD):
        """
        __init__ function for the WallFollowing class

        reference_distance_from_wall is the distance from the wall that the Crazyflie
            should try to keep
        max_forward_speed is the maximum speed the Crazyflie should fly forward
        max_turn_rate is the maximum turn rate the Crazyflie should turn with
        wall_following_direction is the direction the Crazyflie should follow the wall
            (WallFollowingDirection Enum)
        first_run is a boolean that is True if the Crazyflie is in the first run of the
            wall following demo
        prev_heading is the heading of the Crazyflie in the previous state (in rad)
        wall_angle is the angle of the wall in the previous state (in rad)
        around_corner_back_track is a boolean that is True if the Crazyflie is in the
            around corner state and should back track
        state_start_time is the time when the Crazyflie entered the current state (in s)
        ranger_value_buffer is the buffer value for the ranger measurements (in m)
        angle_value_buffer is the buffer value for the angle measurements (in rad)
        range_lost_threshold is the threshold for when the Crazyflie should stop
            following the wall (in m)
        in_corner_angle is the angle the Crazyflie should turn when it is in the corner (in rad)
        wait_for_measurement_seconds is the time the Crazyflie should wait for a
            measurement before it starts the wall following demo (in s)
        front_wall_detect_distance is how close the front sensor must read before a wall
            ahead counts as "reached" (in m) - see the comment where it's assigned below
        init_state is the initial state of the Crazyflie (StateWallFollowing Enum)
        self.state is a shared state variable that is used to keep track of the current
            state of the Crazyflie's wall following
        self.time_now is a shared state variable that is used to keep track of the current (in s)
        """

        self.reference_distance_from_wall = reference_distance_from_wall
        self.max_forward_speed = max_forward_speed
        self.max_turn_rate = max_turn_rate
        self.wall_following_direction_value = float(wall_following_direction.value)
        self.first_run = first_run
        self.prev_heading = prev_heading
        self.wall_angle = wall_angle
        self.around_corner_back_track = around_corner_back_track
        self.state_start_time = state_start_time
        self.ranger_value_buffer = ranger_value_buffer
        self.angle_value_buffer = angle_value_buffer
        self.range_threshold_lost = range_lost_threshold
        self.in_corner_angle = in_corner_angle
        self.wait_for_measurement_seconds = wait_for_measurement_seconds
        self.wall_too_close_distance = wall_too_close_distance
        self.wall_too_far_distance = wall_too_far_distance
        # Kept separate from ranger_value_buffer (default: same
        # reference_distance_from_wall + ranger_value_buffer sum as before,
        # so behavior is unchanged unless a caller explicitly overrides this)
        # because ranger_value_buffer is also reused for two side-range checks
        # (TURN_TO_FIND_WALL's 45-degree approach check, and the corner-align
        # margin in command_turn_around_corner_and_adjust) that have nothing
        # to do with how close the front wall needs to be before it counts as
        # "reached" - tuning one shouldn't silently move the other.
        # 跟ranger_value_buffer分开存(默认值沿用原来
        # reference_distance_from_wall + ranger_value_buffer的和,不显式传入
        # 的话行为不变),因为ranger_value_buffer还被另外两处侧方相关的判断复用
        # (TURN_TO_FIND_WALL里45度接近墙的检查,以及
        # command_turn_around_corner_and_adjust里转角对齐的容差)——这两处跟
        # "前方多近算到墙了"没有关系,调其中一个不该悄悄带动另一个。
        self.front_wall_detect_distance = (
            front_wall_detect_distance
            if front_wall_detect_distance is not None
            else reference_distance_from_wall + ranger_value_buffer)

        self.first_run = True
        self.state = init_state
        self.time_now = 0.0
        self.speed_redux_corner = 3.0
        self.speed_redux_straight = 2.0
        # Front/side range values alone can't reliably tell a genuine
        # 90-degree concave corner (front and side reading two different
        # walls) apart from a mid-wall realignment blip (front and side both
        # reading the SAME wall at a shallow angle) - tried a front/side
        # ratio threshold and it misfired on a legitimate single-wall case
        # with the same ratio as a real corner. Track it behaviorally
        # instead: a short FORWARD_ALONG_WALL leg (see FORWARD_ALONG_WALL
        # transition below) means the small atan-triangulated correction
        # didn't actually align it with the wall, so after a couple of these
        # in a row, force a full 90-degree turn instead of trusting the
        # triangulation again - see TURN_TO_FIND_WALL below.
        # 光看 front/side 的数值没法可靠区分"真正的90度内凹角"(front和side
        # 量的是两面不同的墙)和"沿墙途中的一次普通姿态修正"(front和side
        # 其实量的是同一面墙,只是角度较小)——试过用比值判断,结果在一次
        # 合法的单墙情形里,比值跟真角几乎一样大,照样会误判。改成跟踪实际
        # 表现:如果 FORWARD_ALONG_WALL 这一段飞得很短(见下面
        # FORWARD_ALONG_WALL 的状态转换),说明用三角公式算出的小修正根本
        # 没把方向对齐,连续失败几次以后,就不再相信三角公式,直接强制转
        # 固定90度——见下面 TURN_TO_FIND_WALL 的处理。
        self.short_wall_leg_count = 0
        self.min_wall_leg_seconds = 1.0
        self.corner_escalation_attempts = 2
        # Debounce for FORWARD_ALONG_WALL's "wall lost" exit (see that
        # transition below): reacted to a single tick's side_range with no
        # settle time, and empirically side_range reads inf on essentially
        # every entry into FORWARD_ALONG_WALL (the sensor's own 10Hz update
        # rate lags the 100Hz control loop, so right as TURN_TO_ALIGN_TO_WALL
        # hands off, the reading is often still stale from mid-turn) - this
        # bounced it straight back into the corner-negotiation sequence
        # before it ever got a real straight leg, observed as ~60 state
        # changes across a 2-minute flight instead of a handful of long
        # straight runs. Require the "lost" reading to persist continuously
        # for side_lost_debounce_sec before actually transitioning, giving
        # the sensor a couple of ticks to catch up.
        # FORWARD_ALONG_WALL"墙丢了"这个退出条件(见下面那个状态转换)之前
        # 只看单次读数,没有任何稳定时间——实测发现几乎每次进入
        # FORWARD_ALONG_WALL,side_range读到的都是inf(传感器自己10Hz的
        # 刷新率跟不上100Hz的控制循环,TURN_TO_ALIGN_TO_WALL刚交接的那一刻,
        # 读数往往还是转弯途中的陈旧值),导致还没真正飞出一段直线就被弹回
        # 转角协商流程——实测2分钟飞行里记录了近60次状态切换,而不是应有的
        # 几段长直线。现在要求"丢了"这个读数连续持续
        # side_lost_debounce_sec这么久才真正判定丢失并转换状态,给传感器
        # 留几个周期跟上。
        self.side_lost_debounce_sec = 0.2
        self._side_lost_since = None
        # FIND_CORNER and TURN_TO_FIND_WALL both only rotate in place
        # (velocity_x stays 0 in their action code) and their only exit
        # conditions are sensor-value thresholds - if a full sweep doesn't
        # satisfy them (bad luck in exactly which directions had walls in
        # range that rotation, or a real geometric gap), there is nothing
        # that ever changes and they spin forever (reproduced live: 77s and
        # 100s respectively, pinned to one spot). Every other state either
        # exits on an angle delta (which the constant commanded turn rate
        # guarantees will eventually satisfy, regardless of what the sensors
        # say) or actually translates while it searches. Give these two the
        # same guarantee: once the commanded turn rate would have completed
        # a full circle plus margin, stop trusting the sensor threshold and
        # fall back to FORWARD - already a proven-safe state (it only
        # translates and stops itself the moment front_range gets close), so
        # this reuses existing, tested collision behavior instead of
        # inventing a new blind movement.
        # FIND_CORNER 和 TURN_TO_FIND_WALL 这两个状态都只会原地转
        # (它们的动作代码里velocity_x一直是0),而且退出条件都只看传感器
        # 数值——如果转了一整圈还凑不齐条件(可能是那一圈里恰好哪些方向有墙
        # 在探测范围内不凑巧,也可能是真的遇到几何上的空当),就没有任何东西
        # 会再变化,会永远转下去(实测复现过:分别卡了77秒和100秒,死死钉在
        # 一个位置)。其他状态要么靠角度差退出(以恒定的转速,不管传感器怎么说,
        # 转够时间迟早会满足),要么在搜索的同时真的在平移。给这两个状态同样
        # 的保证:按恒定转速算,转够一整圈再留点余量之后,就不再相信传感器
        # 阈值,退回FORWARD——这是已经验证过安全的状态(只会平移,一旦
        # front_range变近就会自己停下来),相当于复用已经测试过的避障行为,
        # 而不是发明一个新的盲目移动。
        self.spin_timeout_sec = (2 * math.pi / self.max_turn_rate) * 1.3

    # Helper function
    def value_is_close_to(self, real_value, checked_value, margin):
        if real_value > checked_value - margin and real_value < checked_value + margin:
            return True
        else:
            return False

    def wrap_to_pi(self, number):
        if number > math.pi:
            return number - 2 * math.pi
        elif number < -math.pi:
            return number + 2 * math.pi
        else:
            return number

    # Command functions
    def command_turn(self, reference_rate):
        """
        Command the Crazyflie to turn around its yaw axis

        reference_rate and rate_yaw is defined in rad/s
        velocity_x is defined in m/s
        """
        velocity_x = 0.0
        rate_yaw = self.wall_following_direction_value * reference_rate
        return velocity_x, rate_yaw

    def command_align_corner(self, reference_rate, side_range, wanted_distance_from_corner):
        """
        Command the Crazyflie to align itself to the outer corner
            and make sure it's at a certain distance from it

        side_range and wanted_distance_from_corner is defined in m
        reference_rate and rate_yaw is defined in rad/s
        velocity_x is defined in m/s
        """
        if side_range > wanted_distance_from_corner + self.range_threshold_lost:
            rate_yaw = self.wall_following_direction_value * reference_rate
            velocity_y = 0.0
        else:
            if side_range > wanted_distance_from_corner:
                velocity_y = self.wall_following_direction_value * \
                    (-1.0 * self.max_forward_speed / self.speed_redux_corner)
            else:
                velocity_y = self.wall_following_direction_value * (self.max_forward_speed / self.speed_redux_corner)
            rate_yaw = 0.0
        return velocity_y, rate_yaw

    def command_hover(self):
        """
        Command the Crazyflie to hover in place
        """
        velocity_x = 0.0
        velocity_y = 0.0
        rate_yaw = 0.0
        return velocity_x, velocity_y, rate_yaw

    def command_forward_along_wall(self, side_range):
        """
        Command the Crazyflie to fly forward along the wall
            while controlling it's distance to it

        side_range is defined in m
        velocity_x and velocity_y is defined in m/s
        """
        velocity_x = self.max_forward_speed
        velocity_y = 0.0
        # Explicit too-close/too-far bounds instead of a value_is_close_to
        # symmetric band around reference_distance_from_wall - that band used
        # ranger_value_buffer as the margin, and with buffer==reference
        # distance it extended all the way down to side_range=0, making the
        # "too close, back off" correction below unreachable in practice (the
        # drone could drift right up against the wall with no self-correction
        # from this function - only the separate hard SIDE_RANGE_MIN guard in
        # wall_following_multiranger.py would catch it).
        # 用显式的"太近/太远"边界,取代原来围着reference_distance_from_wall
        # 用value_is_close_to凑的对称区间——那个区间拿ranger_value_buffer当
        # 容差,而buffer刚好等于参考距离,导致区间一路延伸到side_range=0,
        # 下面这个"太近就往回修正"实际上永远碰不到(无人机可以一路贴到墙上
        # 都不会被这个函数修正,只能靠wall_following_multiranger.py里那个
        # 独立的硬保护SIDE_RANGE_MIN兜底)。
        if side_range > self.wall_too_far_distance:
            velocity_y = self.wall_following_direction_value * \
                (-1.0 * self.max_forward_speed / self.speed_redux_straight)
        elif side_range < self.wall_too_close_distance:
            velocity_y = self.wall_following_direction_value * (self.max_forward_speed / self.speed_redux_straight)
        return velocity_x, velocity_y

    def command_turn_around_corner_and_adjust(self, radius, side_range):
        """
        Command the Crazyflie to turn around the corner
            and adjust it's distance to the corner

        radius is defined in m
        side_range is defined in m
        velocity_x and velocity_y is defined in m/s
        """
        velocity_x = self.max_forward_speed
        # velocity_x / radius is the yaw rate needed to actually trace a
        # circle of this radius at this speed - unlike every other turn
        # command in this file, this one was never capped at max_turn_rate,
        # so it silently exceeds it whenever max_forward_speed / radius >
        # max_turn_rate (with the current defaults: 0.15 / 0.25 = 0.6 rad/s,
        # 20% over max_turn_rate=0.5). A height crash was observed in this
        # exact state with front/side both comfortably far (not a tight-
        # corner squeeze), which an uncapped, faster-than-tested yaw rate is
        # a more likely explanation for than geometry.
        # velocity_x / radius是要在这个半径、这个速度下真正画出一个圆所需要
        # 的偏航速率——跟本文件里其它所有转弯指令不同,这一处从来没有被
        # max_turn_rate限幅过,所以只要max_forward_speed / radius >
        # max_turn_rate就会悄悄超过它(按目前的默认值:0.15 / 0.25 = 0.6
        # rad/s,比max_turn_rate=0.5高20%)。实测在这个状态下出现过高度骤降,
        # 而且当时前方/侧方都不算近(不是被挤进窄墙角)——一个没限幅、比
        # 测试过的转速更快的偏航速率,比几何空间不够更可能是原因。
        capped_yaw_rate = min(velocity_x / radius, self.max_turn_rate)
        rate_yaw = self.wall_following_direction_value * (-1 * capped_yaw_rate)
        velocity_y = 0.0
        check_distance_wall = self.value_is_close_to(
            self.reference_distance_from_wall, side_range, self.ranger_value_buffer)
        if not check_distance_wall:
            if side_range > self.reference_distance_from_wall:
                velocity_y = self.wall_following_direction_value * \
                    (-1.0 * self.max_forward_speed / self.speed_redux_corner)
            else:
                velocity_y = self.wall_following_direction_value * (self.max_forward_speed / self.speed_redux_corner)
        return velocity_x, velocity_y, rate_yaw

    # state machine helper functions
    def state_transition(self, new_state):
        """
        Transition to a new state and reset the state timer

        new_state is defined in the StateWallFollowing enum
        """
        self.state_start_time = self.time_now
        return new_state

    def adjust_reference_distance_wall(self, reference_distance_wall_new):
        """
        Adjust the reference distance to the wall
        """
        self.reference_distance_from_wall = reference_distance_wall_new

    # Wall following State machine
    def wall_follower(self, front_range, side_range, current_heading,
                      wall_following_direction, time_outer_loop):
        """
        wall_follower is the main function of the wall following state machine.
        It takes the current range measurements of the front range and side range
        sensors, the current heading of the Crazyflie, the wall following direction
        and the current time of the outer loop (the real time or the simulation time)
        as input, and handles the state transitions and commands the Crazyflie to
        to do the wall following.

        front_range and side_range is defined in m
        current_heading is defined in rad
        wall_following_direction is defined as WallFollowingDirection enum
        time_outer_loop is defined in seconds (double)
        command_velocity_x, command_velocity_ y is defined in m/s
        command_rate_yaw is defined in rad/s
        self.state is defined as StateWallFollowing enum
        """

        self.wall_following_direction_value = float(wall_following_direction.value)
        self.time_now = time_outer_loop

        if self.first_run:
            self.prev_heading = current_heading
            self.around_corner_back_track = False
            self.first_run = False

        # -------------- Handle state transitions ---------------- #
        if self.state == self.StateWallFollowing.FORWARD:
            if front_range < self.front_wall_detect_distance:
                self.state = self.state_transition(self.StateWallFollowing.TURN_TO_FIND_WALL)
        elif self.state == self.StateWallFollowing.HOVER:
            print('hover')
        elif self.state == self.StateWallFollowing.TURN_TO_FIND_WALL:
            # Turn until 45 degrees from wall such that the front and side range sensors
            #   can detect the wall
            side_range_check = side_range < (self.reference_distance_from_wall /
                                             math.cos(math.pi/4) + self.ranger_value_buffer)
            front_range_check = front_range < (self.reference_distance_from_wall /
                                               math.cos(math.pi/4) + self.ranger_value_buffer)
            if side_range_check and front_range_check:
                self.prev_heading = current_heading
                # The atan triangulation below only gives the correct wall
                # angle when front_range and side_range are both readings off
                # THE SAME flat wall (a genuine 45-degree approach). At a real
                # concave corner - e.g. flying along wall A and arriving at
                # wall B, which meets it at 90 degrees - front_range measures
                # wall B while side_range still measures wall A: two unrelated
                # surfaces, and the formula produces a bogus small angle
                # (observed live: ~35-38 degrees instead of the real 90).
                # There's no reliable way to tell the two cases apart from
                # front/side alone (tried a ratio threshold - a legitimate
                # single-wall reading hit the same ratio as a real corner and
                # got misclassified). So trust the triangulation first, but
                # track the outcome: if self.short_wall_leg_count shows the
                # last corner_escalation_attempts corrections all produced a
                # too-short FORWARD_ALONG_WALL leg (see that transition
                # below), the triangulated angle clearly isn't working here -
                # force a full 90-degree turn instead, since this maze is
                # built entirely from 90-degree corners.
                # 下面这个反正切三角公式,只有在 front_range 和 side_range
                # 量的是同一面墙(真正以45度接近一面墙)时才算得对。在真实的
                # 内凹角处——比如沿着A墙飞,飞到跟A墙成90度的B墙——
                # front_range量的是B墙,side_range量的还是A墙,是两个不相关
                # 的面,公式会算出一个错误的小角度(实测:35-38度,而不是
                # 真正需要的90度)。光看front/side没法可靠区分这两种情况
                # (试过比值判断,结果一次合法的单墙读数跟真角的比值一样大,
                # 被误判)。所以先相信三角公式,但跟踪结果:如果
                # self.short_wall_leg_count显示最近
                # corner_escalation_attempts次修正都导致FORWARD_ALONG_WALL
                # (见下面那个状态转换)飞得太短,说明三角公式算出来的角度
                # 在这里就是不管用,直接强制转固定90度——反正这个迷宫全是
                # 90度直角。
                if self.short_wall_leg_count >= self.corner_escalation_attempts:
                    self.wall_angle = self.wall_following_direction_value * (math.pi / 2)
                    self.short_wall_leg_count = 0
                else:
                    self.wall_angle = self.wall_following_direction_value * \
                        (math.pi/2 - math.atan(front_range / side_range) + self.angle_value_buffer)
                print(f'TEMP DEBUG wall_angle calc: front={front_range:.3f} side={side_range:.3f} '
                      f'short_wall_leg_count={self.short_wall_leg_count} '
                      f'-> wall_angle={math.degrees(self.wall_angle):.1f}deg prev_heading={math.degrees(self.prev_heading):.1f}deg')
                self.state = self.state_transition(self.StateWallFollowing.TURN_TO_ALIGN_TO_WALL)
            # If went too far in heading and lost the wall, go to find corner.
            if side_range < self.reference_distance_from_wall + self.ranger_value_buffer and \
                    front_range > self.reference_distance_from_wall + self.range_threshold_lost:
                self.around_corner_back_track = False
                self.prev_heading = current_heading
                self.state = self.state_transition(self.StateWallFollowing.FIND_CORNER)
            if self.state == self.StateWallFollowing.TURN_TO_FIND_WALL and \
                    self.time_now - self.state_start_time > self.spin_timeout_sec:
                # Neither condition above fired this whole sweep - see
                # spin_timeout_sec in __init__.
                # 转了这么久,上面两个条件一次都没满足过——见__init__里的
                # spin_timeout_sec。
                self.state = self.state_transition(self.StateWallFollowing.FORWARD)
        elif self.state == self.StateWallFollowing.TURN_TO_ALIGN_TO_WALL:
            align_wall_check = self.value_is_close_to(
                self.wrap_to_pi(current_heading - self.prev_heading), self.wall_angle, self.angle_value_buffer)
            if align_wall_check:
                self._side_lost_since = None
                self.state = self.state_transition(self.StateWallFollowing.FORWARD_ALONG_WALL)
        elif self.state == self.StateWallFollowing.FORWARD_ALONG_WALL:
            # Captured before either branch below calls state_transition()
            # (which resets state_start_time), so both see the same leg
            # duration regardless of which one fires.
            # 在下面任何一个分支调用state_transition()(会重置
            # state_start_time)之前先取好,这样不管哪个分支触发,拿到的都是
            # 同一个这段飞行时长。
            leg_duration = self.time_now - self.state_start_time
            # If side range is out of reach,
            #    end of the wall is reached
            if side_range > self.reference_distance_from_wall + self.range_threshold_lost:
                # Debounce - see side_lost_debounce_sec in __init__. Only
                # commit to "wall lost" once the reading has held for a
                # continuous stretch, not just this one tick.
                # 去抖——见__init__里的side_lost_debounce_sec。只有这个读数
                # 连续持续了一段时间,才真正判定"墙丢了",而不是这一帧一读到
                # 就认。
                if self._side_lost_since is None:
                    self._side_lost_since = self.time_now
                elif self.time_now - self._side_lost_since >= self.side_lost_debounce_sec:
                    # A short leg here means the last TURN_TO_ALIGN_TO_WALL
                    # correction didn't actually line the drone up with the
                    # wall - see short_wall_leg_count in __init__ and its use
                    # in TURN_TO_FIND_WALL above.
                    # 这里飞得短,说明上一次TURN_TO_ALIGN_TO_WALL修正并没有
                    # 真正让方向对齐墙面——见__init__里的short_wall_leg_count,
                    # 以及上面TURN_TO_FIND_WALL对它的使用。
                    if leg_duration < self.min_wall_leg_seconds:
                        self.short_wall_leg_count += 1
                    else:
                        self.short_wall_leg_count = 0
                    self.state = self.state_transition(self.StateWallFollowing.FIND_CORNER)
            else:
                self._side_lost_since = None
            # If front range is small
            #    then corner is reached
            if front_range < self.front_wall_detect_distance:
                if leg_duration < self.min_wall_leg_seconds:
                    self.short_wall_leg_count += 1
                else:
                    self.short_wall_leg_count = 0
                self.prev_heading = current_heading
                self.state = self.state_transition(self.StateWallFollowing.ROTATE_IN_CORNER)
        elif self.state == self.StateWallFollowing.ROTATE_AROUND_WALL:
            if front_range < self.front_wall_detect_distance:
                self.state = self.state_transition(self.StateWallFollowing.TURN_TO_FIND_WALL)
        elif self.state == self.StateWallFollowing.ROTATE_IN_CORNER:
            check_heading_corner = self.value_is_close_to(
                math.fabs(self.wrap_to_pi(current_heading-self.prev_heading)),
                self.in_corner_angle, self.angle_value_buffer)
            if check_heading_corner:
                self.state = self.state_transition(self.StateWallFollowing.TURN_TO_FIND_WALL)
        elif self.state == self.StateWallFollowing.FIND_CORNER:
            if side_range <= self.reference_distance_from_wall:
                self.state = self.state_transition(self.StateWallFollowing.ROTATE_AROUND_WALL)
            elif self.time_now - self.state_start_time > self.spin_timeout_sec:
                # Swept a full circle+ without ever finding a wall close
                # enough - see spin_timeout_sec in __init__.
                # 转了一整圈以上还没找到够近的墙——见__init__里的
                # spin_timeout_sec。
                self.state = self.state_transition(self.StateWallFollowing.FORWARD)
        else:
            # TEMP DEBUG: this branch should be unreachable (every enum member
            # has its own elif above) - if it fires, self.state holds
            # something that failed all 8 identity checks. Print its repr to
            # find out what.
            print(f'UNREACHABLE HOVER FALLBACK: self.state={self.state!r}')
            self.state = self.state_transition(self.StateWallFollowing.HOVER)

        # -------------- Handle state actions ---------------- #
        command_velocity_x_temp = 0.0
        command_velocity_y_temp = 0.0
        command_angle_rate_temp = 0.0

        if self.state == self.StateWallFollowing.FORWARD:
            command_velocity_x_temp = self.max_forward_speed
            command_velocity_y_temp = 0.0
            command_angle_rate_temp = 0.0
        elif self.state == self.StateWallFollowing.HOVER:
            command_velocity_x_temp, command_velocity_y_temp, command_angle_rate_temp = self.command_hover()
        elif self.state == self.StateWallFollowing.TURN_TO_FIND_WALL:
            command_velocity_x_temp, command_angle_rate_temp = self.command_turn(self.max_turn_rate)
            command_velocity_y_temp = 0.0
        elif self.state == self.StateWallFollowing.TURN_TO_ALIGN_TO_WALL:
            if self.time_now - self.state_start_time < self.wait_for_measurement_seconds:
                command_velocity_x_temp, command_velocity_y_temp, command_angle_rate_temp = self.command_hover()
            else:
                command_velocity_x_temp, command_angle_rate_temp = self.command_turn(self.max_turn_rate)
                command_velocity_y_temp = 0.0
        elif self.state == self.StateWallFollowing.FORWARD_ALONG_WALL:
            command_velocity_x_temp, command_velocity_y_temp = self.command_forward_along_wall(side_range)
            command_angle_rate_temp = 0.0
        elif self.state == self.StateWallFollowing.ROTATE_AROUND_WALL:
            # If first time around corner
            #   first try to find the wall again
            # if side range is larger than preffered distance from wall
            if side_range > self.reference_distance_from_wall + self.range_threshold_lost:
                # Keep sweeping in the same direction until the wall is
                # reacquired. (Previously this flipped direction once the
                # sweep passed in_corner_angle, which made it oscillate
                # forever around wide convex corners where the next wall
                # is farther away than range_threshold_lost - it kept
                # reversing before ever sweeping far enough to reach it.)
                command_velocity_y_temp, command_angle_rate_temp = self.command_turn(
                    self.max_turn_rate)
                command_velocity_x_temp = 0.0
            else:
                # continue to turn around corner
                self.prev_heading = current_heading
                self.around_corner_back_track = False
                command_velocity_x_temp, command_velocity_y_temp, command_angle_rate_temp = \
                    self.command_turn_around_corner_and_adjust(
                        self.reference_distance_from_wall, side_range)
        elif self.state == self.StateWallFollowing.ROTATE_IN_CORNER:
            command_velocity_x_temp, command_angle_rate_temp = self.command_turn(self.max_turn_rate)
            command_velocity_y_temp = 0.0
        elif self.state == self.StateWallFollowing.FIND_CORNER:
            # Tried blending in blind forward translation after a stuck
            # timeout here (to escape spinning forever in open space with no
            # wall in sensor range) - it has no obstacle awareness at all, so
            # it drove the drone straight into a wall while still rotating
            # and crashed it (reproduced live: height collapsed to ~0.05m and
            # never recovered for the rest of the flight). A permanent spin
            # is merely inefficient; a blind crash is worse. Reverted - stay
            # pure rotation here, matching the original design.
            # 之前试过在这里加一个"卡住超时就混入前进速度"的兜底(为了避免
            # 飘到没有墙的空地上永远原地转)——但这个动作完全没有避障意识,
            # 结果一边转一边往前冲,直接撞墙摔机了(实测复现:高度塌到
            # 约0.05m,之后整段飞行再也没恢复)。永远原地转顶多是低效,盲目
            # 前冲撞墙更糟。已回退,这里保持跟原设计一样纯旋转。
            command_velocity_y_temp, command_angle_rate_temp = self.command_align_corner(
                -1 * self.max_turn_rate, side_range, self.reference_distance_from_wall)
            command_velocity_x_temp = 0.0
        else:
            # state does not exist, so hover!
            command_velocity_x_temp, command_velocity_y_temp, command_angle_rate_temp = self.command_hover()

        command_velocity_x = command_velocity_x_temp
        command_velocity_y = command_velocity_y_temp
        command_yaw_rate = command_angle_rate_temp

        return command_velocity_x, command_velocity_y, command_yaw_rate, self.state
