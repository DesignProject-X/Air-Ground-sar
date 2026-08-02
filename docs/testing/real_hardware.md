# Real Hardware Testing Guide

Runs the full pipeline across two machines: a real TurtleBot3 ground robot,
a real Intel RealSense camera, and a real Crazyflie for aerial mapping.
A Gazebo-simulated drone can stand in for the Crazyflie — see
[`simulation.md`](simulation.md).

---

## Two machines, two workspaces

- **Ground robot**: its own ROS 2 workspace at `~/tb_ws` on the robot (not
  this repo's `ros2_ws`). Runs `tb3_launcher` (hardware bringup + Nav2, a
  lab-provided package) plus this project's `ground_controller` and
  `custom_msgs`, deployed separately from the base station's copies.
- **Base station**: this repo's `ros2_ws`. Runs the camera, target
  detection, coordinate transform, scheduler, AI planning, the drone
  stack, and the dashboard.

Both machines need the same `ROS_DOMAIN_ID` and matching CycloneDDS config
(`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, plus each machine's
`~/.ros/cyclone_dds.xml` listing the other's IP as a `<Peer>` if the
network has no multicast) to see each other at all.

---

## One-time setup

**On the ground robot**, let its clock be corrected without a password
prompt (these boards have no battery-backed RTC and reset their clock on
every power cycle):

```bash
echo 'tb ALL=(root) NOPASSWD: /usr/bin/date' | sudo tee /etc/sudoers.d/tb-date
sudo chmod 440 /etc/sudoers.d/tb-date
```

**On the base station**, install rosbridge (needed for the dashboard):

```bash
sudo apt install -y ros-humble-rosbridge-suite
```

---

## Starting the system

**Order matters**: the ground robot first, then the base station. The base
station's `clock_sync_client` is a one-shot that needs the robot's
`clock_sync_node` already running to reach it.

```bash
# 1. Ground robot — hardware + Nav2, map receiver, clock sync
ros2 launch ground_controller robot_bringup.launch.py

# 2. Base station — camera, detection, scheduler, planner, rosbridge
ros2 launch sar_bringup base_station_real.launch.py

# 3. UAV (base station)
ros2 launch crazyflie_ros2_multiranger_bringup wall_follower_mapper_real.launch.py

# 4. Dashboard (base station)
python3 ros2_ws/src/sar_bringup/web/launcher.py
# open http://localhost:8080/dashboard.html
```

`launcher.py` serves the dashboard *and* exposes start/stop buttons for
all four items above, so in practice only it needs starting by hand. A
plain `python3 -m http.server` serves the page but leaves those buttons
dead — it cannot run commands.

The dashboard also has buttons to send a preset task command (bypassing
the AI planner) and to reset the scheduler.

---

## Initial pose

AMCL needs an initial pose before it can localize, and it has to match
where the robot is physically placed — a wrong seed leaves AMCL converging
from a large covariance while Nav2 is already driving on it, which shows up
as the robot stopping short of its goal while Nav2 reports success.

`map_receiver_node` auto-publishes one after successfully loading a map, so
the first mission after a robot restart seeds AMCL for free. The defaults
live in that node; the dashboard's "Initial Pose" panel can override them.

**Known gap:** if a mission reuses a map already cached in the scheduler's
memory, no map load happens and nothing re-seeds AMCL. Moving the robot
back to the start between repeated runs needs a manual re-publish. That
cache also survives a robot restart — restart `scheduler_node` alongside
the robot, or it may believe a map is loaded when the robot came back up
empty.

---

## Map handling

`robot_bringup.launch.py`'s `map` argument defaults to an **empty string**.
This is deliberate: with no map, `map_server` comes up cleanly and simply
never publishes until something calls `load_map`, whereas pointing it at a
nonexistent path wedges it in `unconfigured` for good.

That empty default lets the scheduler decide: it first asks the robot
whether a map from an earlier run is already on disk, and only dispatches
the UAV if not. To skip that and force a map:

```bash
ros2 launch ground_controller robot_bringup.launch.py map:=<path to map yaml>
```

---

## Mission flow

A task command (dashboard, natural language or preset) drives the
scheduler through `RECON → MAP_READY → WAITING_TARGET → NAVIGATING → DONE`.
The drone maps the area, the map is injected into the robot's Nav2 stack,
the camera locates the target, and the robot drives to it.

---

# 真机测试指南

跨两台机器运行完整流程:真实的 TurtleBot3 地面机器人、真实的 Intel
RealSense 相机,以及负责空中建图的真实 Crazyflie。也可以用 Gazebo 仿真无人机
替代 Crazyflie——见 [`simulation.md`](simulation.md)。

---

## 两台机器,两个工作空间

- **地面机器人**:机器人上有它自己的 ROS 2 工作空间 `~/tb_ws`(不是本仓库的
  `ros2_ws`)。跑 `tb3_launcher`(硬件启动 + Nav2,实验室提供的包),以及本项目
  的 `ground_controller` 和 `custom_msgs`——这两个包要单独部署到那边,跟基站上
  的副本是两份。
- **基站**:本仓库的 `ros2_ws`。跑相机、目标检测、坐标转换、调度器、AI 规划、
  无人机相关节点,以及网页面板。

两台机器必须使用相同的 `ROS_DOMAIN_ID`,并且 CycloneDDS 配置要匹配
(`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`;如果网络不支持多播,还需要各自的
`~/.ros/cyclone_dds.xml` 里把对方 IP 写成 `<Peer>`),否则两边根本发现不了对方。

---

## 一次性配置

**在地面机器人上**,允许免密修改系统时间(这类板子没有带电池的 RTC,每次断电
重启时钟都会回退):

```bash
echo 'tb ALL=(root) NOPASSWD: /usr/bin/date' | sudo tee /etc/sudoers.d/tb-date
sudo chmod 440 /etc/sudoers.d/tb-date
```

**在基站上**,安装 rosbridge(网页面板需要):

```bash
sudo apt install -y ros-humble-rosbridge-suite
```

---

## 启动系统

**顺序有讲究**:先小车,再基站。基站的 `clock_sync_client` 是一次性的,需要
小车的 `clock_sync_node` 已经在跑才能调到它。

```bash
# 1. 小车 —— 硬件 + Nav2、地图接收、时钟同步
ros2 launch ground_controller robot_bringup.launch.py

# 2. 基站 —— 相机、检测、调度器、规划器、rosbridge
ros2 launch sar_bringup base_station_real.launch.py

# 3. 无人机(基站)
ros2 launch crazyflie_ros2_multiranger_bringup wall_follower_mapper_real.launch.py

# 4. 网页面板(基站)
python3 ros2_ws/src/sar_bringup/web/launcher.py
# 打开 http://localhost:8080/dashboard.html
```

`launcher.py` 既负责发送页面,也提供上面四项的启停按钮,所以实际操作时只需要
手动启动它一个。用普通的 `python3 -m http.server` 也能打开页面,但那些按钮会
全部失效——它没有执行命令的能力。

面板上还有直接下发预设任务(绕过 AI 规划)和重置调度器的按钮。

---

## 初始位姿

AMCL 需要一个初始位姿才能定位,而且它必须跟小车实际摆放的位置一致——播错的话,
AMCL 会带着很大的协方差慢慢收敛,而 Nav2 已经在这份定位上开始行驶了,表现出来
就是小车没到目标就停下、Nav2 却报告成功。

`map_receiver_node` 会在成功加载地图后自动发布一次,所以小车重启后的第一次
任务能免费得到初始位姿。默认值写在那个节点里,面板上的 "Initial Pose" 面板可以
覆盖它。

**已知缺口**:如果某次任务复用的是调度器内存里已缓存的地图,就不会发生地图
加载,也就不会重新播种 AMCL。重复测试时把小车挪回起点,这种情况下需要手动重新
发布一次。这个缓存也不会因为小车重启而失效——小车重启时请连同 `scheduler_node`
一起重启,否则它可能以为地图还在,而小车那边其实是空的。

---

## 地图处理

`robot_bringup.launch.py` 的 `map` 参数默认是**空字符串**。这是有意为之:没有
地图时 `map_server` 能正常启动,只是在有人调用 `load_map` 之前不往外发布;而如果
指向一个不存在的路径,它会永久卡在 `unconfigured` 状态。

这个空默认值把决策权留给了调度器:它会先问小车"之前跑的地图还在不在磁盘上",
只有不在才派无人机。想跳过这一步、强制使用某张地图:

```bash
ros2 launch ground_controller robot_bringup.launch.py map:=<地图 yaml 路径>
```

---

## 任务流程

一条任务指令(面板上的自然语言或预设按钮)会驱动调度器走完
`RECON → MAP_READY → WAITING_TARGET → NAVIGATING → DONE`:无人机建图,地图注入
小车的 Nav2,相机定位目标,小车驶向目标。
