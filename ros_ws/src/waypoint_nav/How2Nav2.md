# Waypoint Navigation with Nav2

## Overview

Waypoint navigation runs on top of the existing SLAM system — it does not replace or modify it. The pipeline is:

```
landmark_based_slam.py
    │
    ├─ /slam/pose  ──►  slam_tf_bridge.py  ──►  TF: map → base_link  (Nav2 needs this)
    │
    └─ /slam/lidar_map  ──►  topic_tools relay  ──►  /map  (Nav2 costmap)

Nav2  ──►  /cmd_vel  ──►  rover
 ▲
 └─ run_waypoints.py  (loads pose_history_save.txt and sends goals)
```

SLAM publishes its pose and map as usual. Two small bridge processes translate those into the interface Nav2 expects, without touching the original topics.

---

## Prerequisites

**1. Record a route first.**
Drive the robot manually with SLAM running. When you `Ctrl+C` the SLAM node it saves `pose_history_*.txt`. Rename or copy the file you want to follow to:

```
waypoint_nav/pose_history_save.txt
```

**2. Install Nav2 and topic_tools** (one-time, on the robot):

```bash
sudo apt install ros-humble-nav2-bringup ros-humble-topic-tools
```

---

## Launching

**Setup** (one-time, after making package changes):
```bash
cd /home/copperbottoms/code/Copperbottoms/ros_ws
colcon build --packages-select waypoint_nav
source install/setup.bash
```

**Terminal 1 — SLAM stack** (sensors, EKF, lidar, camera):
```bash
cd /home/copperbottoms/code/Copperbottoms/ros_ws/src/SLAM
ros2 launch SLAM_dependency_launch.py
```

** This launch should no longer be needed **
**Terminal 2 — landmark_based stack** (sensors, EKF, lidar, camera):
```bash
cd /home/copperbottoms/code/Copperbottoms/ros_ws/src/SLAM
python3 landmark_based_slam.py
```

**Terminal 3 — Nav2 + waypoint navigation:**
```bash
ros2 launch waypoint_nav waypoint_nav_launch.py
```

**OR SIMPLE NAV:**
```bash
python3 simple_waypoint_launch.py
ros2 run simple_waypoint simple_waypoint_launch.py
```

Terminal 2 handles everything else automatically:
- Starts the TF bridge
- Starts the map relay
- Launches Nav2
- Waits for Nav2 to be ready, then begins following waypoints

---

## Files

| File | Purpose |
|------|---------|
| `launch/waypoint_nav_launch.py` | Main launch file for Nav2 + bridges |
| `run_waypoints.py` | Loads waypoints and sends them to Nav2 |
| `nav2_params.yaml` | Nav2 configuration (speeds, costmap, planner) |
| `waypoint_extraction.py` | Utility to preview/downsample a pose history file |
| `pose_history_save.txt` | The recorded route the robot will follow |
| `../slam_tf_bridge.py` | Converts `/slam/pose` → TF `map→base_link` |

---

## Tuning

**Change how densely waypoints are sampled** from the recorded route — edit `every_n` in `run_waypoints.py`:
```python
waypoints_raw = load_waypoints(..., every_n=4)  # keep every 4th pose
```

**Change robot speed** — edit `nav2_params.yaml`:
```yaml
FollowPath:
  desired_linear_vel: 0.3  # m/s
```

**Change goal tolerance** (how close the robot must get before moving to the next waypoint) — add to `nav2_params.yaml` under `waypoint_follower`:
```yaml
waypoint_follower:
  ros__parameters:
    stop_on_failure: false
    waypoint_task_executor_plugin: "wait_at_waypoint"
```
