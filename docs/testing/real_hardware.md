# Real Hardware Testing Guide

Runs the full pipeline against a real TurtleBot3 ground robot and a real
Intel RealSense camera (no `fake_agents` ground/camera stubs). The UAV side
still uses `fake_uav_node` unless a real or simulated Crazyflie flight is
wired in (see "UAV recon options" below) — this guide covers the ground
robot + base station half, which is what's actually been tested.

---

## Two machines, two workspaces

- **Ground robot**: its own separate ROS 2 workspace at `~/tb_ws` on the
  robot itself (not this repo's `ros2_ws`). Runs `tb3_launcher` (hardware
  bringup + Nav2, a lab-provided package) plus this project's
  `ground_controller` and `custom_msgs` packages, deployed there
  separately from the base station's copies.
- **Base station**: this repo's `ros2_ws`. Runs the camera, target
  detection, coordinate transform, scheduler, AI planning, and the web
  dashboard's WebSocket gateway.

Both machines need the same `ROS_DOMAIN_ID` and matching CycloneDDS config
(`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, plus each machine's
`~/.ros/cyclone_dds.xml` listing the other machine's IP as a `<Peer>` if
your network doesn't support multicast discovery) to see each other at all.

---

## One-time setup

**On the ground robot**, allow the robot's own clock to be corrected
without a password prompt (needed because most of these boards have no
battery-backed RTC and reset their clock on every power cycle):

```bash
echo 'tb ALL=(root) NOPASSWD: /usr/bin/date' | sudo tee /etc/sudoers.d/tb-date
sudo chmod 440 /etc/sudoers.d/tb-date
```

**On the base station**, install rosbridge (only needed for the web
dashboard):

```bash
sudo apt install -y ros-humble-rosbridge-suite
```

---

## Starting the system

```bash
# On the ground robot
ros2 launch ground_controller robot_bringup.launch.py
```

Starts `tb3_launcher`'s hardware + Nav2 bringup, plus `map_receiver_node`
(map injection receiver) and `clock_sync_node` (lets the base station push
a correct time to this robot over a ROS 2 service instead of SSH). No
`map:=` argument by default — see "Map handling" below for why.

```bash
# On the base station
ros2 launch sar_bringup base_station_real.launch.py
ros2 run fake_agents fake_uav_node
```

`base_station_real.launch.py` starts: the RealSense camera (depth-aligned),
`target_detector_node`, `coordinate_bridge_node`, `ground_controller_node`
(the Nav2 action client), `scheduler_node`, a one-shot `clock_sync_client`,
`rosbridge_websocket`, and `planner_node`. `fake_uav_node` isn't included —
run it separately (see "UAV recon options" below).

```bash
# Dashboard (base station)
cd ros2_ws/src/sar_bringup/web
python3 -m http.server 8080
# open http://localhost:8080/dashboard.html
```

The dashboard auto-connects to `ws://localhost:9090`. Green connection dot
= rosbridge reachable.

---

## Initial pose

AMCL needs an initial pose before it can localize. The dashboard has an
"Initial Pose" panel (pre-filled with the last-used test coordinates) with
a publish button — place the robot at that physical spot and click it, or
override the x/y/yaw fields first if the robot's starting spot changed.

**This does *not* always happen automatically.** `map_receiver_node`
auto-publishes an initial pose right after it successfully loads a map (so
the very first mission after a robot restart, or one that needed aerial
recon, seeds AMCL for free) — but if a mission simply reuses a map already
cached in the scheduler's memory (`current_map`, no service call to the
robot at all), nothing re-triggers this. Moving the robot back to the start
position between repeated test runs, in that case, needs a manual
re-publish.

**Known gap:** `current_map` is only cleared by restarting `scheduler_node`
itself — it does *not* get invalidated just because the ground robot
restarted. If only the robot restarts (e.g. it lost power), the scheduler
may still believe a map is loaded when the robot's `map_server` actually
came back up empty. Restart `scheduler_node` alongside the robot to avoid
this.

---

## Map handling

`robot_bringup.launch.py`'s `map` argument defaults to an **empty string**,
not a map file. This is deliberate, confirmed empirically:

- With no map loaded, `map_server` still configures and activates
  normally — it just never publishes on `/map` until something calls
  `/map_server/load_map`.
- Pointed at a path that doesn't exist, `map_server` throws inside its
  `on_configure()` callback and gets stuck in the `unconfigured` lifecycle
  state permanently (this specific build doesn't implement an error-state
  recovery) — every later service call, including `load_map`, then hangs
  or fails.

So: empty default → `map_server` comes up cleanly with nothing loaded →
`scheduler_node`'s `current_map` (populated from `/map`) correctly stays
unset → a task command that finds no cached map calls the ground robot's
`load_existing_map` service first (does `drone_map.yaml` already exist on
disk from an earlier run?) and only dispatches `fake_uav_node` if that also
comes back empty. This decision deliberately lives in the scheduler, not as
a static launch-time argument — see the git history around
`map_receiver_node.py`'s `load_existing_map` service for the reasoning.

To skip recon and force a specific map instead:
```bash
ros2 launch ground_controller robot_bringup.launch.py map:=<path to a valid map yaml>
```

---

## UAV recon options

Three ways `/uav/dispatch` can get a response, in order of how well-tested
each is in this project:

1. **`fake_uav_node`** (tested, this guide's default) — reads
   `sar_bringup/maps/maze_map.yaml` and reports it as the recon result
   after a simulated 3s flight. Run standalone: `ros2 run fake_agents
   fake_uav_node`.
2. **Real Crazyflie** — `cf_controller/cf_mission_node.py` bridges
   `UavDispatch` → takeoff → wall-following → target detection →
   `MapResult`, wired via `crazyflie_ros2_multiranger_bringup`'s
   `wall_follower_mapper_real.launch.py`. Message types match what the
   scheduler expects, but this path hasn't been exercised against real
   flight hardware in this project yet.
3. **Simulated Crazyflie (Gazebo)** — the flight/mapping side exists
   (`wall_follower_mapper_simulation.launch.py`) but nothing bridges it to
   `/uav/dispatch` / `/uav/map_result` yet; `simple_mapper_multiranger`
   publishes a plain `OccupancyGrid`, not wrapped in `MapResult`. A
   simulation equivalent of `cf_mission_node.py` would need to be written
   before this path is usable with the scheduler.

---

## Known hardware gotchas

- **Camera needs true USB 3.0.** A RealSense D435/D436 streaming
  simultaneous color+depth needs USB3 SuperSpeed bandwidth. Plugged into a
  USB2 port or a USB2-only hub, the device still enumerates and the driver
  node still starts, but frames never arrive
  ("Frames didn't arrived within 5 seconds"). Check `lsusb -t` for the
  negotiated speed next to the camera's `uvcvideo` entries — `5000M`+ is
  USB3, `480M` is USB2.
- **`ros2 launch` on the robot doesn't reliably kill its children.**
  Ctrl-C-ing the launch wrapper can leave `nav2_container`,
  `robot_state_publisher`, `ld08_driver`, etc. still running. Before
  relaunching, check with `ps aux | grep -E 'nav2_container|turtlebot3'`
  and kill any leftovers by PID — a lingering old AMCL/map_server instance
  running alongside a fresh one produces confusing, hard-to-diagnose TF
  errors (e.g. `coordinate_bridge_node` reporting stale `map → base_link`
  transforms that never resolve).
- **No RTC on the robot.** Its clock resets on every power cycle.
  `clock_sync_client` (base station, one-shot, runs as part of
  `base_station_real.launch.py`) pushes the correct time over to the
  robot's `clock_sync_node` automatically — but only if the robot's launch
  is already up when the base station launch starts. If the robot was
  power-cycled after the base station was already running, re-run just the
  client: `ros2 run ground_controller clock_sync_client`.
- **The robot's `ros2` CLI daemon goes stale after processes are
  killed/restarted a lot.** Symptom: `xmlrpc.client.Fault:
  RuntimeError:!rclpy.ok()` from any `ros2 <verb>` command run on the
  robot. Fix: `ros2 daemon stop && ros2 daemon start`.
