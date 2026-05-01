# Copperbottoms
advanced robo repo

## Run Landmark Slam
**Launch the following in separate terminals**
open two terminals 
ros2 run joy joy_node --ros-args --params-file joystick.yaml
ros2 run teleop_twist_joy teleop_node --ros-args --params-file joystick.yaml

run SUPPORTING NODES
open a terminal
cd /home/copperbottoms/code/Copperbottoms/ros_ws/src/SLAM
ros2 launch SLAM_dependency_launch.py

open a terminal
cd /home/copperbottoms/code/Copperbottoms/ros_ws/src/SLAM
python3 landmark_based_slam.py

Ctl-c 
writes robot poses to pose_history.txt
draws landmarks and poses to map_pose_history.jpg
saves lidar-based occupancy grid to last_lidar_grid.npy




## Stopsign Detection
A brief overview on the stopsign detetion node.
Uses a online-sourced cascade model with open-CV to scan for stop-signs.
When stop-sign detected stops, pauses all other velocity commands.
After wait period begins to allow driving again and ignores stop sign reading briefly
### How to Run: Stopsign Detetion
**Launch the following across seperate terminals**
- Need rover_node running: "ros2 run robo_rover rover_node"
- Need realsense camera running: "ros2 launch robo_realsense realsense_launch"

- **Run stopsign detecct with**  "ros2 run stop_sign_detect stop_sign_detect"


## Sensor Visualization
**Launch the following across seperate terminals**
### How to Run: Sensor Visualization
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

## Reverse Driving
**Launch the following accross separate terminals ***
ros2 run robo_rover rover_node to get rover node running
ros2 launch rplidar_ros rplidar_a1_launch.py to get the lidar running
python3 reverse_driving.py to run the script
*STAND CLEAR OF THE SIDES AND FRONT*
