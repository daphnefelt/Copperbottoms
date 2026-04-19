# ROS 2 Latency Monitoring System - User Guide

## Overview

Non-invasive monitoring infrastructure for measuring end-to-end latency in the RTES rover line following system. All monitoring code is contained in the `amy_test` package and does not modify any existing nodes.

**System Validated:** Camera capture (424×240) → Vision processing → Motor actuation

**Requirements:**
- Vision processing: <50ms (camera → cmd_vel)
- Motor actuation: <20ms (cmd_vel → motor response)
- Total end-to-end: <100ms (camera → motor actuation)

---

## Quick Start

### 1. Build the System

```bash
cd ~/code/Copperbottoms/ros_ws
source /opt/ros/galactic/setup.bash
colcon build --packages-select custom_messages amy_test --symlink-install
source install/setup.bash
```

### 2. Launch Modes

#### **Standard Monitoring** (Background Logging)
Monitor latency and log to CSV files for offline analysis:

```bash
ros2 launch amy_test latency_monitoring.launch.py
```

Logs saved to: `~/performance_logs/`

#### **Dashboard Mode** (Live Display)
Real-time console dashboard with latency and rate statistics:

```bash
ros2 launch amy_test latency_monitoring.launch.py mode:=dashboard
```

#### **Automated Test** (30 Second Validation)
Automated pass/fail test against requirements:

```bash
ros2 launch amy_test latency_monitoring.launch.py mode:=test
```

Custom test duration:
```bash
ros2 launch amy_test latency_monitoring.launch.py mode:=test test_duration:=60
```

---

## Architecture

### Monitoring Nodes

1. **topic_rate_monitor** - Measures publication rates and jitter
   - Monitors: `/camera/color/image_raw`, `/cmd_vel`, `/imu/accel`
   - Publishes: `/monitoring/rate_stats`
   - Window: 5 seconds

2. **vision_latency_monitor** - Camera to cmd_vel latency
   - Correlates camera frames with cmd_vel commands
   - Publishes: `/monitoring/vision_latency`
   - Deadline: 50ms
   
3. **motor_latency_monitor** - cmd_vel to motor actuation latency
   - Detects IMU acceleration changes following velocity commands
   - Publishes: `/monitoring/motor_latency`
   - Deadline: 20ms

4. **latency_aggregator** - Combines vision + motor latency
   - Calculates total end-to-end latency
   - Publishes: `/monitoring/total_latency`
   - Deadline: 100ms

5. **performance_logger** - CSV logging
   - Logs all statistics to timestamped CSV files
   - Directory: `~/performance_logs/`

6. **latency_dashboard** - Live console display (optional)
   - Real-time latency and rate monitoring
   - Refreshes every 2 seconds

7. **automated_latency_test** - Automated validation (optional)
   - Runs for specified duration
   - Generates pass/fail report

---

## Custom Messages

### LatencyStats.msg
```
std_msgs/Header header
string measurement_type      # "vision", "motor", or "total"
float64 mean_latency_ms
float64 min_latency_ms
float64 max_latency_ms
float64 stddev_latency_ms
uint32 sample_count
float64 miss_rate_percent    # % exceeding deadline
float64 deadline_ms
```

### RateStats.msg
```
std_msgs/Header header
string topic_name
float64 mean_rate_hz
float64 min_rate_hz
float64 max_rate_hz
float64 mean_jitter_ms
float64 max_jitter_ms
uint32 message_count
float64 window_duration_s
```

---

## Usage Examples

### Example 1: Baseline Measurement

Measure system performance while rover follows a line:

```bash
# Terminal 1: Start line following
ros2 launch robo_rover line_follow_V2.launch.py

# Terminal 2: Start monitoring (background logging)
ros2 launch amy_test latency_monitoring.launch.py

# Drive rover on line track for 1-2 minutes
# Press Ctrl+C when done

# Analyze results
cd ~/performance_logs
ls -ltr  # Find latest log files
```

### Example 2: Real-time Dashboard

Monitor performance during tuning:

```bash
# Terminal 1: Start line following
ros2 launch robo_rover line_follow_V2.launch.py

# Terminal 2: Launch dashboard
ros2 launch amy_test latency_monitoring.launch.py mode:=dashboard

# Watch live statistics while rover runs
```

### Example 3: Automated Validation

Quick pass/fail test:

```bash
# Terminal 1: Start line following
ros2 launch robo_rover line_follow_V2.launch.py

# Terminal 2: Run 30-second automated test
ros2 launch amy_test latency_monitoring.launch.py mode:=test

# Test runs and generates report to /tmp/latency_test_report_*.txt
```

### Example 4: Custom Configuration

Adjust monitoring parameters:

```bash
ros2 launch amy_test latency_monitoring.launch.py \
  mode:=monitoring \
  log_dir:=/data/rover_logs \
  test_duration:=120
```

---

## Monitoring Topics

Subscribe to these topics for custom analysis:

```bash
# Vision latency statistics
ros2 topic echo /monitoring/vision_latency

# Motor actuation latency
ros2 topic echo /monitoring/motor_latency

# Total end-to-end latency
ros2 topic echo /monitoring/total_latency

# Topic publication rates
ros2 topic echo /monitoring/rate_stats
```

---

## Performance Logs

Logs are saved to `~/performance_logs/` (configurable via `log_dir` parameter):

- `rover_performance_latency_YYYYMMDD_HHMMSS.csv` - Latency measurements
- `rover_performance_rate_YYYYMMDD_HHMMSS.csv` - Rate statistics

### Latency CSV Format:
```
timestamp,measurement_type,mean_latency_ms,min_latency_ms,max_latency_ms,
stddev_latency_ms,sample_count,miss_rate_percent,deadline_ms
```

### Rate CSV Format:
```
timestamp,topic_name,mean_rate_hz,min_rate_hz,max_rate_hz,
mean_jitter_ms,max_jitter_ms,message_count,window_duration_s
```

---

## Expected Performance

Based on Phase 0 FIFO testing (424×240 resolution):

| Component | Expected Latency | Deadline | Status |
|-----------|------------------|----------|--------|
| Vision (camera → cmd_vel) | 30-40ms | 50ms | ✓ GOOD |
| Motor (cmd_vel → actuation) | 10-15ms | 20ms | ✓ GOOD |
| **Total (camera → actuation)** | **45-55ms** | **100ms** | ✓ EXCELLENT |

**Baseline FIFO Results:**
- Hough line detection at 424×240: 18ms mean execution time
- Expected ROS overhead: 12-22ms (message passing, callbacks, scheduling)

---

## Troubleshooting

### "Insufficient latency samples" Warning

**Vision latency monitor:**
- Cause: Camera not publishing or cmd_vel not being generated
- Solution: Verify line_follow node is running and rover is on line track

**Motor latency monitor:**
- Cause: Rover not moving or IMU acceleration changes too small
- Solution: Drive rover with varying velocity commands (accelerate/decelerate)

### Build Errors

**Missing custom_messages:**
```bash
cd ~/code/Copperbottoms/ros_ws
colcon build --packages-select custom_messages --allow-overriding custom_messages
source install/setup.bash
```

**Missing dependencies:**
```bash
sudo apt install ros-galactic-std-msgs
```

### No Dashboard Output

Check terminal is not headless:
```bash
echo $DISPLAY  # Should not be empty
```

For headless systems, use `mode:=monitoring` instead of `mode:=dashboard`.

---

## Integration with Existing System

**Non-invasive design:**
- All monitoring code in `amy_test` package
- No modifications to `line_follow_V2.py`, `rover_node.py`, or `ros_stream.py`
- Can be launched/stopped independently
- Zero overhead when not running

**To integrate monitoring into existing launch files:**

Edit your main launch file to include:
```python
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# Include latency monitoring
latency_monitoring = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        os.path.join(get_package_share_directory('amy_test'), 
                     'launch', 'latency_monitoring.launch.py')
    ]),
    launch_arguments={'mode': 'monitoring'}.items()
)

return LaunchDescription([
    # Your existing nodes...
    latency_monitoring,
])
```

---

## Next Steps

### Phase 1 Complete ✓
- Custom timing messages created
- All monitoring nodes implemented
- Launch files configured
- System built and verified

### Phase 2: Baseline Measurements (Ready to Run)

1. Run automated test to validate requirements:
   ```bash
   ros2 launch amy_test latency_monitoring.launch.py mode:=test
   ```

2. Collect extended baseline data:
   ```bash
   ros2 launch amy_test latency_monitoring.launch.py mode:=monitoring
   # Run for 5-10 minutes on various line tracks
   ```

3. Analyze CSV logs for patterns:
   - Latency vs. rover speed
   - Latency vs. line curvature
   - Impact of lighting conditions
   - Performance degradation over time

### Phase 3: Optimization (If Needed)

If total latency exceeds 100ms:
- Tune camera resolution/frame rate
- Optimize vision processing algorithm
- Reduce ROS message overhead
- Adjust control loop frequencies

### Phase 4: Production Validation

- Long-term stability testing (hours)
- Stress testing (rapid direction changes)
- Environmental testing (different lighting, line types)
- Integration with PID tuning

---

## Support

For issues or questions:
1. Check logs in `~/performance_logs/`
2. Review automated test report in `/tmp/latency_test_report_*.txt`
3. Verify all nodes are running: `ros2 node list`
4. Check topic publication: `ros2 topic hz /camera/color/image_raw`

---

**Author:** Amy Test Monitoring Infrastructure  
**Version:** 1.0  
**Date:** April 18, 2026  
**Location:** `~/code/Copperbottoms/ros_ws/src/amy_test/`
