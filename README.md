# Air-Ground Collaborative Exploration & Target Rescue
### Via Embodied Gateway and Dual-Agent System

> SUTD EPD · MTD (Robotics & Automation) · EPD 30.537 Design Project

An air-ground collaborative search-and-rescue system for unknown indoor maze environments.
The operator issues a single natural language command; the system autonomously completes
the full pipeline from aerial mapping to ground navigation and target rescue.

---

## System Architecture (Three Layers)

```
Human Operator
      │  Natural language command
      ▼
┌─────────────────────────────────────────────────┐
│ AI Planning Layer      (LangGraph + Gemini)      │  NL → task plan → TaskCommand
├─────────────────────────────────────────────────┤
│ Scheduling & Gateway Layer  (Python FSM)         │  Sole scheduler · agent dispatch
│                                                  │  map injection · fault recovery
├─────────────────────────────────────────────────┤
│ Execution Layer        (ROS 2 Humble)            │
│   ├── UAV Agent   : CrazyFlie + D436             │  2D SLAM mapping · YOLO detection
│   └── Ground Agent: TurtleBot3 + 2D LiDAR        │  Nav2 navigation · obstacle avoidance
└─────────────────────────────────────────────────┘
```

> See `docs/architecture/` for full diagram.

## Hardware

| Module | Device | Role |
|--------|--------|------|
| UAV Platform | CrazyFlie | Aerial recon · 2D map generation |
| Depth Camera | Intel RealSense D436 | UAV-side 2D SLAM + target detection |
| Ground Robot | TurtleBot3 (+ 2D LiDAR) | Autonomous navigation · target rescue |
| Embedded AI | Jetson Orin Nano | Onboard inference · YOLO detection |

## Software Stack

- ROS 2 Humble
- Python 3.10+
- LangGraph + Gemini API (AI planning layer)
- Custom ROS 2 message types (`custom_msgs`)

## Repository Structure

```
air-ground-sar/
├── docs/                    # Architecture diagrams · presentations · reports · demo materials
│   ├── architecture/
│   ├── presentations/
│   ├── reports/
│   ├── demo/
│   └── testing/             # Simulation testing guides
├── ros2_ws/
│   └── src/
│       ├── ai_planning/     # AI planning layer (LangGraph graph, prompts, structured output)
│       ├── scheduling/      # Scheduling & gateway layer (Python FSM, agent dispatch, map injection)
│       ├── fake_agents/     # Simulation stubs for UAV and ground robot (hardware-free testing)
│       ├── custom_msgs/     # Custom ROS 2 message definitions (TaskCommand, MapResult, TaskItem)
│       └── sar_bringup/     # Launch files and parameter config for the full system
└── scripts/                 # Environment setup / one-shot launch scripts
```

## Quick Start

```bash
# 1. Install Python dependencies
python3 -m venv venv && source venv/bin/activate
pip install -r requirements.txt

# 2. Build
cd ros2_ws
colcon build
source install/setup.bash

# 3. Launch (simulation — no hardware required)
export GEMINI_API_KEY=your_key_here
ros2 launch sar_bringup simulate_system.launch.py
```

## Environment Variables

```bash
export GEMINI_API_KEY=your_gemini_api_key_here
export ROS_DOMAIN_ID=0
```

## Demo

> Video links and key screenshots in `docs/demo/`

> Simulation testing guide (no hardware required): [`docs/testing/simulation.md`](docs/testing/simulation.md)

---

## 系统简介

陆空协同搜救系统：操作员输入一条自然语言指令，系统自动完成从空中建图到地面导航的全流程，用于实验室物理迷宫的未知环境搜救。

## 系统架构（三层）

```
操作员
      │  自然语言指令
      ▼
┌─────────────────────────────────────────────────┐
│ AI Planning Layer      (LangGraph + Gemini)      │  自然语言 → 任务规划 → TaskCommand
├─────────────────────────────────────────────────┤
│ Scheduling & Gateway Layer  (Python FSM)         │  唯一调度权威 · agent 派发
│                                                  │  地图注入 · 故障恢复
├─────────────────────────────────────────────────┤
│ Execution Layer        (ROS 2 Humble)            │
│   ├── UAV Agent   : CrazyFlie + D436             │  2D SLAM 建图 · YOLO 目标检测
│   └── Ground Agent: TurtleBot3 + 2D LiDAR        │  Nav2 自主导航 · 障碍物规避
└─────────────────────────────────────────────────┘
```

> 完整架构图见 `docs/architecture/`

## 硬件平台

| 模块 | 设备 | 职责 |
|------|------|------|
| UAV 平台 | CrazyFlie | 航拍侦查 · 2D 地图生成 |
| 深度相机 | Intel RealSense D436 | UAV 端 2D SLAM 建图 + 目标检测 |
| 地面机器人 | TurtleBot3（含 2D LiDAR）| 自主导航 · 目标搜救 |
| 嵌入式算力 | Jetson Orin Nano | 板载 AI 推理 · YOLO 检测 |

## 软件栈

- ROS 2 Humble
- Python 3.10+
- LangGraph + Gemini API（AI 规划层）
- 自定义 ROS 2 消息类型（`custom_msgs`）

## 项目结构

```
air-ground-sar/
├── docs/                    # 架构图 · 演示文稿 · 技术报告 · 演示素材
│   ├── architecture/
│   ├── presentations/
│   ├── reports/
│   ├── demo/
│   └── testing/             # 仿真测试指南
├── ros2_ws/
│   └── src/
│       ├── ai_planning/     # AI 规划层（LangGraph 图、提示词、结构化输出）
│       ├── scheduling/      # 调度 & 网关层（Python 状态机、agent 派发、地图注入）
│       ├── fake_agents/     # UAV 与地面机器人仿真桩（无需硬件即可测试）
│       ├── custom_msgs/     # 自定义 ROS 2 消息定义（TaskCommand、MapResult、TaskItem）
│       └── sar_bringup/     # 全系统 launch 文件与参数配置
└── scripts/                 # 环境配置 / 一键启动脚本
```

## 快速开始

```bash
# 1. 安装 Python 依赖
python3 -m venv venv && source venv/bin/activate
pip install -r requirements.txt

# 2. 编译
cd ros2_ws
colcon build
source install/setup.bash

# 3. 启动（仿真模式，无需硬件）
export GEMINI_API_KEY=your_key_here
ros2 launch sar_bringup simulate_system.launch.py
```

## 环境变量

```bash
export GEMINI_API_KEY=your_gemini_api_key_here
export ROS_DOMAIN_ID=0
```

## 演示

> 视频链接与截图见 `docs/demo/`

> 仿真测试指南（无需硬件）：[`docs/testing/simulation.md`](docs/testing/simulation.md)

---

> EPD 30.537 Design Project · SUTD
