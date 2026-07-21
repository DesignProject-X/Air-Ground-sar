# Real Hardware Testing Guide

Runs the full pipeline against a real TurtleBot3 ground robot and a real
Intel RealSense camera (no `fake_agents` ground/camera stubs). The UAV side
defaults to `fake_uav_node`, but a real simulated Crazyflie flight
(Gazebo) can stand in instead — see "UAV recon options" below, which is
now tested end-to-end (ground robot + real camera + simulated aerial
recon all running together).

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
2. **Simulated Crazyflie (Gazebo)** (tested end-to-end with a real ground
   robot + real camera) — `cf_controller/cf_mission_node_sim.py` bridges
   `UavDispatch` to the same Gazebo Crazyflie + `simple_mapper_multiranger`
   + `wall_following_multiranger` stack used for pure-simulation testing
   (see [`simulation.md`](simulation.md)), reporting a real `MapResult`
   built from actual simulated flight instead of a static map file. Run
   standalone:
   ```bash
   ros2 launch cf_controller sim_uav_recon.launch.py fake_target_signal:=false
   ```
   `fake_target_signal:=false` is required here — by default this launch
   also fakes a `/camera/target_pose` after mapping (useful when testing
   the sim UAV in isolation against `fake_agents`, see `simulation.md`),
   which would race the real camera's own detection and could make the
   scheduler skip `WAITING_TARGET` before the real camera ever gets a
   chance to fire. With it off, this node only ever reports the map;
   target detection is left entirely to `target_detector_node`. See
   "Hybrid test: simulated UAV + real ground robot + real camera" below
   for the full procedure.
3. **Real Crazyflie** — `cf_controller/cf_mission_node.py` bridges
   `UavDispatch` → takeoff → wall-following → target detection →
   `MapResult`, wired via `crazyflie_ros2_multiranger_bringup`'s
   `wall_follower_mapper_real.launch.py`. Message types match what the
   scheduler expects, but this path hasn't been exercised against real
   flight hardware in this project yet.

---

## Hybrid test: simulated UAV + real ground robot + real camera

Runs the Gazebo-simulated Crazyflie for aerial mapping while the ground
robot and target detection are both real hardware — the closest this
project gets to a full mission without an actual flight-ready Crazyflie.

```bash
# On the ground robot (unchanged)
ros2 launch ground_controller robot_bringup.launch.py

# On the base station: real camera + scheduler + planner (unchanged)
ros2 launch sar_bringup base_station_real.launch.py

# Also on the base station (needs GPU for Gazebo's rendering - run
# elsewhere on the same ROS_DOMAIN_ID if the base station's GPU is busy)
ros2 launch cf_controller sim_uav_recon.launch.py fake_target_signal:=false
```

Place the robot, publish its initial pose (see "Initial pose" above), then
trigger a mission the same way as any other test (operator command via the
dashboard, or a direct `TaskCommand` publish - see
[`simulation.md`](simulation.md)'s Test Case 1 for the exact message).

Expected sequence: the simulated drone climbs and wall-follows the maze for
`cf_mission_node_sim`'s `mapping_duration_sec` (120s default), reports
`MapResult` with no fake target attached, the scheduler moves
`RECON → MAP_READY` and injects the map into the real ground robot, the
robot starts real Nav2 navigation, and at some point the real camera
(`target_detector_node`) detects the target and publishes
`/camera/target_pose` — independently of the mapping timer, since the two
are decoupled (mapping completion and target detection are unrelated
events that just happen to both feed the scheduler). If the camera happens
to already have the target in view when the mission starts (e.g. a fixed
overhead view of a static scene), it may detect it well before the drone
finishes mapping - the scheduler caches this and skips `WAITING_TARGET`
the instant `MAP_READY` is reached, so the robot can start moving very
soon after the map arrives. That's expected behavior (see `scheduler_node
._proceed_after_map_ready`), not a bug — if you want to test the "robot
searches after arriving" path instead, keep the target out of the camera's
view until the mission is already in `NAVIGATING`.

**`scheduler_node.RECON_TIMEOUT_S` is 140s** specifically to leave margin
above the sim UAV's 120s default mapping duration (plus a few seconds of
climb/handoff before the mapping timer even starts) - if you shorten
`mapping_duration_sec` for a faster test, this doesn't need to change, but
if you lengthen it past ~130s, `RECON_TIMEOUT_S` needs to grow with it or
the scheduler will retry before the drone finishes.

**Known gotcha - `/cmd_vel` collision.** `cf_hover_sim.py` and
`wall_following_multiranger.py` (both third-party, from
`crazyflie_ros2_multiranger`) hardcode their Twist publisher as the bare,
unnamespaced `/cmd_vel`, assuming the simulated drone is the only robot on
the network. On real hardware both machines share one `ROS_DOMAIN_ID` (see
"Two machines, two workspaces" above) - and a real TurtleBot3 base
controller also listens on that same bare `/cmd_vel`. Reproduced live: the
instant the sim drone started flying, the real ground robot started moving
too, with the scheduler still in `RECON` and no map ever sent - the sim
drone's raw hover/wall-following Twist was reaching the real robot's
motors directly. `sim_uav_recon.launch.py` now remaps `/cmd_vel` to
`/crazyflie/hover_cmd_vel` for every node that touches it (including
`control_services`, started via the included
`crazyflie_simulation.launch.py`) via a `launch_ros.actions.SetRemap` at
the top of the launch description - `control_services`'s own
`/crazyflie/cmd_vel` output (a different, already-namespaced topic, what
actually bridges to Gazebo) is untouched. If this ever gets edited, verify
with `ros2 topic info /cmd_vel` right after launch - it should report
`Unknown topic` (nothing publishing or subscribing to the bare name at
all).

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
