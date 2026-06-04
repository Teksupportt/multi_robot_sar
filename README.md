# Multi-Robot Autonomous Search and Rescue System

A full-stack robotics simulation of an autonomous search-and-rescue mission built with ROS 2, PX4 SITL, Nav2, and SLAM Toolbox. A coordinated swarm of 4 autonomous drones scouts a search area while a ground rover navigates autonomously to detected targets. The system operates under degraded communications and enforces security constraints. All nodes are written in C++.

## System Overview

| Component | Role |
|---|---|
| Drone swarm (x4) | Autonomous flight, sector-based search, target detection |
| Ground rover | Autonomous navigation via Nav2 + SLAM to detected targets |
| Swarm coordinator | Multi-agent coordination, detection clustering, rover dispatch |
| Base station | Mission control, visualization, logging |

## Tech Stack

| Technology | Purpose |
|---|---|
| ROS 2 Humble | Communication backbone |
| PX4 SITL | Drone flight control, offboard mode |
| Gazebo Classic | Physics simulation |
| Nav2 + SLAM Toolbox | Autonomous ground navigation |
| Micro XRCE-DDS | PX4 ↔ ROS 2 bridge |
| SROS2 | Security layer (Phase 6) |
| tc/netem | RF degradation simulation (Phase 5) |

## Project Phases

| Phase | Status | Description |
|---|---|---|
| 1 — System Skeleton | ✅ Complete | ROS 2 communication backbone, custom message types |
| 2 — Navigation + Localization | ✅ Complete | Nav2 + SLAM Toolbox, autonomous rover navigation |
| 3 — Flight Control + Swarm | ✅ Complete | PX4 SITL, offboard control, 4-drone coordinated swarm |
| 4 — Perception | 🔧 In Progress | Computer vision pipeline, onboard inference |
| 5 — Benchmarking | ⬜ Planned | PX4 vs ArduPilot, A* vs RRT*, SLAM vs GPS |
| 6 — Comm Stress Testing | ⬜ Planned | tc/netem impairments, QoS tuning |
| 7 — Security | ⬜ Planned | SROS2, enclaves, access control |

## Workspace Structure

    src/
    ├── sar_interfaces/     # Custom ROS 2 messages (TargetDetection, SwarmStatus)
    ├── sar_drone/          # PX4 offboard control, state machine, swarm behavior
    ├── sar_rover/          # Nav2 action client, SLAM integration
    └── sar_base_station/   # Swarm coordinator, detection clustering, rover dispatch

## Dependencies

- ROS 2 Humble
- PX4 Autopilot (SITL)
- Micro XRCE-DDS Agent
- Nav2 and SLAM Toolbox
- Turtlebot3 packages
- Gazebo Classic

## License

MIT
