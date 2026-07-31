#!/usr/bin/env python3

""" This simple mapper is loosely based on both the bitcraze cflib point cloud example
 https://github.com/bitcraze/crazyflie-lib-python/blob/master/examples/multiranger/multiranger_pointcloud.py
 and the webots epuck simple mapper example:
 https://github.com/cyberbotics/webots_ros2

 Originally from https://github.com/knmcguire/crazyflie_ros2_experimental/
 """

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from std_srvs.srv import SetBool

import tf_transformations
import math
import numpy as np

# Sized for the 1.5x1.2m maze (with margin) instead of the original 20x20m
# default, which was leftover from a much larger reference environment - at
# 0.1m resolution the maze's 2cm-thick walls and 0.6m corridors were barely
# resolvable (a wall was a fifth of a cell wide).
# 0.03m turned out too fine in practice: the multiranger only casts 4 fixed
# rays per scan (front/back/left/right), so the map has always had small gaps
# between what each ray actually swept - at 0.1m those gaps were sub-pixel
# and invisible, but at 0.03m they're several pixels wide and show up as
# visible unmapped patches along the flown path. 0.05m is a middle ground:
# still resolves the walls far better than the original 0.1m, without
# exaggerating the beam-coverage gaps as much.
# The grid is centered on wherever the drone happens to take off (map/odom
# origin), not on the maze's geometric center - on real hardware the takeoff
# spot is rarely dead-center, so 2.0m (only 1.0m of margin from takeoff point
# to any edge) proved too tight: a wall further than 1.0m from takeoff falls
# outside the grid entirely. 3.0m gives 1.5m of margin in every direction,
# comfortably covering the 1.5x1.2m maze even from a corner takeoff spot.
# 按1.5x1.2m的迷宫实际尺寸(留一点边界)调整,而不是原来抄自更大场地的20x20m
# 默认值——0.1m分辨率下,2cm厚的墙只有格子边长的五分之一,基本分辨不出来。
# 0.03m实测偏细了:multiranger每次扫描只发4条固定方向的射线(前后左右),地图
# 本来就存在射线扫不到的缝隙——0.1m分辨率下这些缝隙不到一个像素,看不出来,
# 但0.03m下缝隙有好几个像素宽,飞过的路径上会看到明显没建到图的空白。0.05m
# 是折中:比原来0.1m清楚很多,又不会把射线覆盖不到的缝隙放大得太明显。
# 这个栅格是以无人机起飞点(map/odom原点)为中心的,不是以迷宫的几何中心为
# 中心——真机测试起飞点很少刚好在正中间,2.0m(起飞点到边缘只有1.0m余量)
# 实测偏紧:只要一面墙离起飞点超过1.0m就会落在栅格外。3.0m能在每个方向留出
# 1.5m余量,就算起飞点靠近迷宫一角,也能把整个1.5x1.2m的迷宫装下。
GLOBAL_SIZE_X = 3.0
GLOBAL_SIZE_Y = 3.0
MAP_RES = 0.08


class SimpleMapperMultiranger(Node):
    def __init__(self):
        super().__init__('simple_mapper_multiranger')
        self.declare_parameter('robot_prefix', '/crazyflie')
        robot_prefix = self.get_parameter('robot_prefix').value

        self.odom_subscriber = self.create_subscription(
            Odometry, robot_prefix + '/odom', self.odom_subscribe_callback, 10)
        self.ranges_subscriber = self.create_subscription(
            LaserScan, robot_prefix + '/scan', self.scan_subscribe_callback, 10)
        self.position = [0.0, 0.0, 0.0]
        self.angles = [0.0, 0.0, 0.0]
        self.ranges = [0.0, 0.0, 0.0, 0.0]
        self.range_max = 3.5

        self.tfbr = StaticTransformBroadcaster(self)
        t_map = TransformStamped()
        t_map.header.stamp = self.get_clock().now().to_msg()
        t_map.header.frame_id = 'map'
        t_map.child_frame_id =robot_prefix +'/odom'
        t_map.transform.translation.x = 0.0
        t_map.transform.translation.y = 0.0
        t_map.transform.translation.z = 0.0
        self.tfbr.sendTransform(t_map)

        self.position_update = False

        # Lets a mission node freeze mapping right before a landing (e.g.
        # cf_mission_node stopping the search on reaching a checkpoint/
        # timeout) so the brief attitude/position disturbance of the descent
        # doesn't get drawn into the map - this node otherwise has no notion
        # of "searching" vs "landing" and would keep mapping through both.
        # 让任务节点在降落前先把建图冻结住(比如cf_mission_node因为到达检查点
        # /超时而停止搜索的时候)——这个节点本身不知道"正在搜索"还是"正在
        # 降落"的区别,不加这个开关的话,降落过程中短暂的姿态/位置扰动也会
        # 被继续画进地图里。
        self.mapping_active = True
        self.set_mapping_active_srv = self.create_service(
            SetBool, robot_prefix + '/set_mapping_active', self._set_mapping_active_cb)

        self.map = [-1] * int(GLOBAL_SIZE_X / MAP_RES) * \
            int(GLOBAL_SIZE_Y / MAP_RES)
        self.map_publisher = self.create_publisher(OccupancyGrid, robot_prefix + '/map',
                                                   qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST,))

        self.get_logger().info(f"Simple mapper set for crazyflie " + robot_prefix +
                               f" using the odom and scan topic")

    def bresenham_line(self, x0, y0, x1, y1):
        """
        Bresenham's line algorithm implementation
        Returns a list of (x, y) coordinates from (x0, y0) to (x1, y1)
        """
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        x, y = x0, y0
        
        while True:
            points.append((x, y))
            
            if x == x1 and y == y1:
                break
                
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
                
        return points

    def odom_subscribe_callback(self, msg):
        self.position[0] = msg.pose.pose.position.x
        self.position[1] = msg.pose.pose.position.y
        self.position[2] = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        euler = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.angles[0] = euler[0]
        self.angles[1] = euler[1]
        self.angles[2] = euler[2]
        self.position_update = True

    def _set_mapping_active_cb(self, request, response):
        self.mapping_active = request.data
        self.get_logger().info(
            f"Mapping {'resumed' if self.mapping_active else 'frozen'} via set_mapping_active")
        response.success = True
        return response

    def scan_subscribe_callback(self, msg):
        if not self.mapping_active:
            return
        self.ranges = msg.ranges
        self.range_max = msg.range_max
        data = self.rotate_and_create_points()

        points_x = []
        points_y = []
        #
        if self.position_update is False:
            return
        # Grid cells beyond [0, map_width)/[0, map_height) are outside the
        # tracked area entirely - a plain Python list silently wraps negative
        # indices around to the far end instead of raising, so without this
        # check a point beyond one edge gets drawn onto the opposite edge
        # instead of being dropped (reproduced on real hardware: a wall
        # further than the grid's half-size from the takeoff point showed up
        # mirrored onto the opposite side of the map).
        # 超出[0, map_width)/[0, map_height)范围的格子已经不在建图范围内了——
        # 普通Python列表对负数索引是悄悄从末尾环绕回来的,不会报错,所以没有
        # 这个检查的话,超出一侧边界的点会被画到地图的另一侧,而不是被丢弃
        # (真机上复现过:离起飞点距离超过栅格半径的墙,会被镜像画到地图的
        # 另一侧)。
        map_width = int(GLOBAL_SIZE_X / MAP_RES)
        map_height = int(GLOBAL_SIZE_Y / MAP_RES)
        for i in range(len(data)):
            #self.get_logger().info(f"Point {i} {data[i]}")
            # The grid's cell (0, 0) sits at world (-GLOBAL_SIZE/2, -GLOBAL_SIZE/2)
            # (see msg.info.origin below), so converting a world coordinate to
            # an array index means ADDING half the grid size, not subtracting
            # it - subtracting always lands on a negative index for any point
            # near the map center. That negative index used to silently wrap
            # around to the correct-looking cell via Python's negative-list-
            # indexing (list[-n] == list[len-n]), which is what made this look
            # like it worked before - but it only wrapped to the *correct*
            # place by coincidence for points close enough to center, and
            # wrapped to the *wrong* place once a point was far enough out
            # (the original bug report). Adding the bounds check above without
            # fixing this sign made every point compute a negative (now
            # rejected) index, hence the fully empty map.
            # 栅格的(0,0)格对应世界坐标(-GLOBAL_SIZE/2, -GLOBAL_SIZE/2)(见下面
            # 的msg.info.origin),所以世界坐标转数组索引应该是"加上"半个栅格
            # 尺寸,不是"减去"——减法对任何靠近地图中心的点算出来都是负数索引。
            # 这个负数索引以前能"看起来正常",是因为Python负数列表索引会悄悄
            # 从末尾环绕回来(list[-n] == list[len-n]),对离中心足够近的点刚好
            # 环绕到了"看起来对"的格子——但这只是离中心近时的巧合,点一旦超出
            # 这个范围,环绕到的就是错误的格子(也就是最初报告的那个bug)。上面
            # 加了边界检查却没修这个符号,导致每个点算出来都是负数索引、都被
            # 边界检查挡掉,建出来的图就完全是空的。
            point_x = int((data[i][0] + GLOBAL_SIZE_X / 2.0) / MAP_RES)
            point_y = int((data[i][1] + GLOBAL_SIZE_Y / 2.0) / MAP_RES)
            points_x.append(point_x)
            points_y.append(point_y)
            position_x_map = int(
                (self.position[0] + GLOBAL_SIZE_X / 2.0) / MAP_RES)
            position_y_map = int(
                (self.position[1] + GLOBAL_SIZE_Y / 2.0) / MAP_RES)
            for line_x, line_y in self.bresenham_line(position_x_map, position_y_map, point_x, point_y):
                if 0 <= line_x < map_width and 0 <= line_y < map_height:
                    self.map[line_y * map_width + line_x] = 0
            if 0 <= point_x < map_width and 0 <= point_y < map_height:
                self.map[point_y * map_width + point_x] = 100

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = MAP_RES
        msg.info.width = int(GLOBAL_SIZE_X / MAP_RES)
        msg.info.height = int(GLOBAL_SIZE_Y / MAP_RES)
        msg.info.origin.position.x = - GLOBAL_SIZE_X / 2.0
        msg.info.origin.position.y = - GLOBAL_SIZE_Y / 2.0
        msg.data = self.map
        self.map_publisher.publish(msg)

    def rotate_and_create_points(self):
        data = []
        o = self.position
        roll = self.angles[0]
        pitch = self.angles[1]
        yaw = self.angles[2]
        r_back = self.ranges[0]
        r_right = self.ranges[1]
        r_front = self.ranges[2]
        r_left = self.ranges[3]

        if (r_left < self.range_max and r_left != 0.0 and math.isinf(r_left) == False):
            left = [o[0], o[1] + r_left, o[2]]
            data.append(self.rot(roll, pitch, yaw, o, left))

        if (r_right < self.range_max and r_right != 0.0 and math.isinf(r_right) == False):
            right = [o[0], o[1] - r_right, o[2]]
            data.append(self.rot(roll, pitch, yaw, o, right))

        if (r_front < self.range_max and r_front != 0.0 and math.isinf(r_front) == False):
            front = [o[0] + r_front, o[1], o[2]]
            data.append(self.rot(roll, pitch, yaw, o, front))

        if (r_back < self.range_max and r_back != 0.0 and math.isinf(r_back) == False):
            back = [o[0] - r_back, o[1], o[2]]
            data.append(self.rot(roll, pitch, yaw, o, back))

        return data

    def rot(self, roll, pitch, yaw, origin, point):
        cosr = math.cos((roll))
        cosp = math.cos((pitch))
        cosy = math.cos((yaw))

        sinr = math.sin((roll))
        sinp = math.sin((pitch))
        siny = math.sin((yaw))

        roty = np.array([[cosy, -siny, 0],
                        [siny, cosy, 0],
                        [0, 0,    1]])

        rotp = np.array([[cosp, 0, sinp],
                        [0, 1, 0],
                        [-sinp, 0, cosp]])

        rotr = np.array([[1, 0,   0],
                        [0, cosr, -sinr],
                        [0, sinr,  cosr]])

        rotFirst = np.dot(rotr, rotp)

        rot = np.array(np.dot(rotFirst, roty))

        tmp = np.subtract(point, origin)
        tmp2 = np.dot(rot, tmp)
        return np.add(tmp2, origin)


def main(args=None):

    rclpy.init(args=args)
    simple_mapper_multiranger = SimpleMapperMultiranger()
    rclpy.spin(simple_mapper_multiranger)
    rclpy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
