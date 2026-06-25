# Simulation Testing Guide

Tests the AI Planning Layer and Scheduling & Gateway Layer end-to-end
without real hardware. The `fake_agents` package provides stub UAV and
ground robot nodes that simulate hardware responses with fixed delays.

---

## Prerequisites

```bash
cd ~/ros2_ws
colcon build --packages-select ai_planning scheduling fake_agents custom_msgs sar_bringup
source install/setup.bash
export GEMINI_API_KEY=your_key_here
```

---

## Starting the System

```bash
ros2 launch sar_bringup simulate_system.launch.py
```

This starts all four nodes in a single terminal:
- `scheduler_node` — Scheduling & Gateway Layer
- `fake_uav_node` — simulates 3s aerial mapping delay
- `fake_ground_node` — simulates 4s navigation delay
- `planner_node` — AI Planning Layer

For the feedback replanning test (Test Case 3), individual nodes are needed — see below.

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
IDLE --> RECON
RECON --> MAP_READY
MAP_READY --> NAVIGATING
NAVIGATING --> DONE
```

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

Tests the feedback loop: scheduler exhausts retries → notifies planner →
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

The scheduler will retry `aerial_recon` three times (each after
`RECON_TIMEOUT_S = 60s`), then publish a failure message to
`/planner/feedback`. The planner picks this up and replans — the LLM
should drop `aerial_recon` and select an alternative (e.g. `request_backup`)
given that the UAV is reported as unresponsive.

Expected output (planner, after ~180s):
```
Feedback received: Mission failed: aerial_recon timed out ... Replanning...
Task command published: ... 1 task(s).
  Task 1/1: [request_backup] -> [scheduler]
```

---

## Topic Reference

| Topic | Type | Direction |
|---|---|---|
| `/operator/command` | `std_msgs/String` | Operator → Planner |
| `/scheduler/task_command` | `custom_msgs/TaskCommand` | Planner → Scheduler |
| `/uav/dispatch` | `std_msgs/Bool` | Scheduler → UAV |
| `/uav/map_result` | `custom_msgs/MapResult` | UAV → Scheduler |
| `/map` | `nav_msgs/OccupancyGrid` | Scheduler → Nav2 |
| `/ground/goal_pose` | `geometry_msgs/PoseStamped` | Scheduler → Ground |
| `/goal_reached` | `std_msgs/Bool` | Ground → Scheduler |
| `/nav_status` | `std_msgs/String` | Ground → Scheduler |
| `/planner/feedback` | `std_msgs/String` | Scheduler → Planner |
