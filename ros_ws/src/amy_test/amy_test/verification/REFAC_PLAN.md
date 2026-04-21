# Refactor Plan: Split tape_contour_rt.py into Three ROS 2 Nodes

## Overview
Split the current tape_contour_rt.py into three modular ROS 2 nodes for maintainability and clarity:

### 1. tape_vision_node.py
- Subscribes: /camera/color/image_raw
- Publishes: /tape_contour (custom message), /tape_debug/image (optional)
- Responsibilities: Image acquisition, color filtering, contour detection, publish tape info

### 2. tape_control_node.py
- Subscribes: /tape_contour, /turn_right
- Publishes: /cmd_vel
- Responsibilities: PID control, basic lost tape logic, publish velocity commands

### 3. tape_recovery_node.py
- Subscribes: /tape_contour, /cmd_vel
- Publishes: /cmd_vel (override if in recovery)
- Responsibilities: Advanced recovery, gap/dead-end logic, override velocity if needed

### 4. Custom Message: TapeContour.msg
- bool found
- float32 center_x
- float32 center_y
- float32 angle
- float32 area

### 5. Launch and Build
- Update launch files to start all three nodes
- Update CMakeLists.txt and package.xml to include new nodes and custom message

## Steps
1. Define TapeContour.msg in amy_test/msg/
2. Implement tape_vision_node.py
3. Implement tape_control_node.py
4. Implement tape_recovery_node.py
5. Update launch and build files
6. Test the pipeline
