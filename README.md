# Copperbottoms
advanced robo repo


## Stopsign Detection
A brief overview on the stopsign detetion node.
Uses a online-sourced cascade model with open-CV to scan for stop-signs.
When stop-sign detected stops, pauses all other velocity commands.
After wait period begins to allow driving again and ignores stop sign reading briefly.
### How to Run: Stopsign Detetion
**Launch the following across separate terminals**
- Need rover_node running: "ros2 run robo_rover rover_node"
- Need realsense camera running: "ros2 launch robo_realsense realsense_launch"

- **Run stopsign detecct with**  "ros2 run stop_sign_detect stop_sign_detect"


## Sensor Visualization
A brief overview on sensor visualization.
Subscribes to a series of nodes that provide 
### How to Run: Sensor Visualization
**Launch the following across separate terminals**
- Need rover_node running: "ros2 run robo_rover rover_node"
- Need land_mark slam running *This provides pose esimates information*
    - Slam Dependencies: 
    cd /home/copperbottoms/code/Copperbottoms/ros_ws/src/SLAM
    ros2 launch SLAM_dependency_launch.py
    - Landmark Launch:
    cd /home/copperbottoms/code/Copperbottoms/ros_ws/src/SLAM
    python3 landmark_based_slam.py
-  **Run sensor visualization with** "ros2 run visualize_sensors visualize_sensors"
*Note: need graphics enabled, in SSH connection*

