# Simulation Testing Guide

Tests the AI Planning Layer and Scheduling & Gateway Layer end-to-end
without real hardware. The `fake_agents` package provides stub UAV, camera,
and ground robot nodes that simulate hardware responses with fixed delays.

---

## Prerequisites

```bash
cd ~/ros2_ws
colcon build --packages-select ai_planning scheduling fake_agents custom_msgs sar_bringup
source install/setup.bash
export GOOGLE_API_KEY=your_key_here
```

---

## Starting the System

```bash
ros2 launch sar_bringup simulate_system.launch.py
```

This starts five nodes in a single terminal:
- `scheduler_node` — Scheduling & Gateway Layer
- `fake_uav_node` — simulates a 3s aerial mapping delay; reports
  `sar_bringup/maps/maze_map.yaml` as the recon result (not a blank
  placeholder), so the simulated map has real wall/free/unknown content
- `fake_camera_node` — simulates target detection: starts a 2s timer as
  soon as the UAV is dispatched, then publishes a fixed target pose
  directly to `/camera/target_pose` (there's no real camera or
  coordinate-bridge node in the simulated pipeline, so this publishes
  straight in the `map` frame rather than going through a raw-detection →
  transform step)
- `fake_ground_node` — simulates a 4s navigation delay; also stands in for
  `map_receiver_node`'s `save_map` and `load_existing_map` services (see
  "Topic/Service Reference" below) so the scheduler's map-injection and
  map-reuse checks have something to call instead of timing out. It never
  actually has a saved map, so the simulated pipeline always goes through
  `aerial_recon` rather than skipping it via reuse.
- `planner_node` — AI Planning Layer

For the feedback replanning test (Test Case 3), individual nodes are needed — see below.

### A more realistic alternative: simulated flight instead of `fake_uav_node`

```bash
ros2 launch sar_bringup simulate_system_gazebo_uav.launch.py
```

Same pipeline, but swaps `fake_uav_node` (a static pre-baked map, 3s fake
delay) for a real Gazebo Crazyflie that actually climbs, wall-follows the
maze, and builds a `MapResult` from genuine simulated flight data via
`cf_controller/cf_mission_node_sim.py`. `fake_camera_node` is dropped too —
`cf_mission_node_sim` fakes the target-detection signal itself (tied to
real flight progress via a `mapping_duration_sec` timer, not a flat delay),
and running both would double-publish `/camera/target_pose`. Everything
else (`scheduler_node`, `fake_ground_node`, `planner_node`) is unchanged.

Mapping takes `cf_mission_node_sim.py`'s `mapping_duration_sec` (120s
default, not overridden here) so Test Case 1 below takes noticeably longer
than with `fake_uav_node`'s 3s - `scheduler_node`'s `RECON_TIMEOUT_S = 140s`
leaves margin above that for climb/handoff before the mapping timer even
starts. Trigger it the same way. Requires GPU capable of running Gazebo's
rendering.

---

## Test Cases

### 1. Full SAR Mission

Expected sequence: `aerial_recon → map_injection → navigate_to_target`

```bash
ros2 topic pub --once /operator/command std_msgs/msg/String \
  "{data: 'Conduct a full search and rescue in zone B'}"
```

Expected output (scheduler):
```
No usable saved map on ground robot (...). Dispatching aerial recon.
IDLE --> RECON
Target detected during RECON. Will skip WAITING_TARGET once map is ready.
RECON --> MAP_READY
MAP_READY --> NAVIGATING
NAVIGATING --> DONE
```

The "Target detected during RECON" line shows up because `fake_camera_node`'s
2s detection delay is shorter than `fake_uav_node`'s 3s mapping delay, so the
target is usually already known by the time the map comes back — this
exercises the scheduler's early-detection path (skip `WAITING_TARGET`
entirely) rather than the more common real-hardware timing where detection
happens after the map is already loaded.

---

### 2. Recon Only

Expected sequence: `aerial_recon` only, no ground robot dispatched.

```bash
ros2 topic pub --once /operator/command std_msgs/msg/String \
  "{data: 'Recon zone B only'}"
```

Expected output (scheduler):
```
IDLE --> RECON
RECON --> MAP_READY
Recon-only mission. No navigation required.
MAP_READY --> DONE
```

---

### 3. Feedback Replanning (LangGraph Loop)

Tests the feedback loop: a step fails → scheduler notifies planner →
planner replans via LangGraph using the execution feedback.

**Setup:** start scheduler and planner only, without fake agents.

```bash
# Terminal 1
ros2 run scheduling scheduler_node

# Terminal 2
ros2 run ai_planning planner_node
```

```bash
ros2 topic pub --once /operator/command std_msgs/msg/String \
  "{data: 'Conduct a full search and rescue in zone B'}"
```

With no `fake_ground_node` running, `load_existing_map` isn't available
either, so the scheduler logs a warning and falls straight through to
`aerial_recon` (same net effect as the fake ground node reporting "no saved
map", just without a service to call). Nothing ever answers the dispatch,
so after `RECON_TIMEOUT_S = 140s` the scheduler enters `FAILED` and
publishes a failure message to `/planner/feedback`. It stays in `FAILED`
rather than re-dispatching — publish to `/scheduler/reset` to return it to
`IDLE`. The planner picks the feedback up and replans; the LLM should drop
`aerial_recon` and select an alternative (e.g. `request_backup`) given that
the UAV is reported as unresponsive.

Expected output (planner, after ~140s):
```
Feedback received: Mission failed: aerial_recon timed out ... Replanning...
Task command published: ... 1 task(s).
  Task 1/1: [request_backup] -> [scheduler]
```

---

## Topic/Service Reference

| Topic/Service | Type | Direction |
|---|---|---|
| `/operator/command` | `std_msgs/String` | Operator → Planner |
| `/scheduler/task_command` | `custom_msgs/TaskCommand` | Planner → Scheduler |
| `/uav/dispatch` | `custom_msgs/UavDispatch` | Scheduler → UAV |
| `/uav/map_result` | `custom_msgs/MapResult` | UAV → Scheduler |
| `save_map` (service) | `custom_msgs/srv/SaveMap` | Scheduler → Ground (map injection) |
| `load_existing_map` (service) | `std_srvs/srv/Trigger` | Scheduler → Ground (map reuse check) |
| `/map` | `nav_msgs/OccupancyGrid` | Ground (map_server) → Scheduler (subscribes; scheduler no longer publishes this directly - see `scheduling/scheduler_node.py`'s `current_map`) |
| `/camera/target_pose` | `geometry_msgs/PoseStamped` | Camera → Scheduler |
| `/ground/goal_pose` | `geometry_msgs/PoseStamped` | Scheduler → Ground |
| `/goal_reached` | `std_msgs/Bool` | Ground → Scheduler |
| `/nav_status` | `std_msgs/String` | Ground → Scheduler |
| `/planner/feedback` | `std_msgs/String` | Scheduler → Planner |
| `/scheduler/state` | `std_msgs/String` | Scheduler → dashboard (current FSM state, TRANSIENT_LOCAL) |
| `/planner/last_plan` | `std_msgs/String` (JSON) | Planner → dashboard (last plan incl. per-task reasons) |

On real hardware, `save_map`/`load_existing_map` are served by
`ground_controller/map_receiver_node.py` and `/map` is served by Nav2's
`map_server` running on the ground robot itself — see
[`real_hardware.md`](real_hardware.md).
