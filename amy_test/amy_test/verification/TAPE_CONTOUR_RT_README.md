# Real-Time Tape Contour Detection Node

## Overview
This node performs real-time tape detection using color filtering and contour analysis, based on the FIFO_Hough.c template. It subscribes to camera images, applies transformations, and publishes visualization for RViz 2.

**Key Features:**
- Real-time processing at 320×240 resolution
- SCHED_FIFO priority scheduling (requires sudo)
- Color-based tape detection (configurable BGR values)
- Contour detection with OpenCV
- Direction calculation based on contour angle and center position
- Timing performance monitoring (70ms deadline)
- RViz 2 visualization support
- Optional motor control output

## Tape Color Configuration
The node currently uses these BGR values for tape detection:
```python
tape_color = [164, 108, 7]      # BGR format
color_tolerance = [50, 50, 90]
```

**Note:** These values appear to be for brown/orange tape. For blue tape, you would need different values (e.g., [255, 50, 50] for bright blue in BGR).

## Installation

1. Build the package:
```bash
cd ~/code/Copperbottoms/ros_ws
colcon build --packages-select amy_test
source install/setup.bash
```

## Usage

### Basic Testing (No Motor Control)
```bash
ros2 launch amy_test tape_contour_rt.launch.py
```

### With Motor Control Enabled
```bash
ros2 launch amy_test tape_contour_rt.launch.py enable_motor_control:=true
```

### With Custom Parameters
```bash
ros2 launch amy_test tape_contour_rt.launch.py \
    enable_motor_control:=true \
    forward_speed:=0.3 \
    max_turn:=1.2 \
    deadline_ms:=80.0
```

### Running with Real-Time Priority (Recommended)
For best performance, run with sudo to enable SCHED_FIFO scheduling:
```bash
sudo -E ros2 launch amy_test tape_contour_rt.launch.py
```

## Visualization in RViz 2

1. Start RViz 2:
```bash
rviz2
```

2. Add an Image display:
   - Click "Add" → "By topic"
   - Select `/tape_debug/image`
   - Set transport to "raw" or "compressed"

3. The visualization shows:
   - **Left panel:** Original image with:
     - Green contour outline
     - Red center point
     - Purple direction vector (shows tape angle)
     - Yellow center line (image midpoint)
     - Turn command and timing info
   - **Right panel:** Binary mask after color filtering

## Topics

### Subscribed
- `/camera/color/image_raw` - Input camera images (Image)

### Published
- `/tape_debug/image` - Debug visualization (Image) - **View this in RViz 2**
- `/cmd_vel` - Motor commands (Twist) - Only if `enable_motor_control:=true`
- `/tape_debug/timing` - Timing statistics (String)

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `image_topic` | `/camera/color/image_raw` | Camera input topic |
| `cmd_vel_topic` | `/cmd_vel` | Motor command output topic |
| `debug_image_topic` | `/tape_debug/image` | Visualization output topic |
| `forward_speed` | `0.25` | Forward speed (m/s) |
| `max_turn` | `1.0` | Maximum turn rate (rad/s) |
| `deadline_ms` | `70.0` | Real-time deadline (ms) |
| `target_width` | `320` | Processing image width |
| `target_height` | `240` | Processing image height |
| `min_contour_area` | `100` | Minimum contour area (pixels) |
| `publish_timing` | `true` | Publish timing statistics |
| `enable_motor_control` | `false` | Enable motor commands (safety) |

## Algorithm Details

### Color Filtering
1. Convert to BGR color space
2. Apply color threshold based on `tape_color ± color_tolerance`
3. Morphological operations (closing + opening) to remove noise

### Contour Detection
1. Find all external contours in binary mask
2. Filter by minimum area (`min_contour_area`)
3. Select largest valid contour

### Direction Calculation
The steering command is computed from:
- **Lateral Error:** Contour center position relative to image center
- **Angle Error:** Contour orientation (should be vertical for tape)

```python
turn = -(kp_lateral * lateral_error + kp_angle * angle_error)
```

Default weights:
- `kp_lateral = 0.8` - Position-based steering
- `kp_angle = 0.3` - Orientation-based steering

## Performance Monitoring

The node tracks and reports:
- Processing time per frame (ms)
- Average, min, max processing times
- Deadline miss count and percentage
- Real-time performance rating

Example output:
```
Frame 100: 12.45ms (avg=13.21, min=11.23, max=18.76) | Misses: 2/100 (2.0%) | Tape: True | Turn: -0.32
```

Performance ratings:
- ✓ **Excellent:** < 1% misses
- ✓ **Good:** < 5% misses
- ⚠ **Marginal:** < 10% misses
- ✗ **Poor:** ≥ 10% misses

## Adjusting Tape Color

To change the tape color being detected, modify the node source file:

```python
# In tape_contour_rt.py, around line 60:
self.tape_color = np.array([B, G, R])  # BGR values
self.color_tolerance = np.array([B_tol, G_tol, R_tol])
```

**Finding the right values:**
1. Capture a sample image of your tape
2. Use a color picker tool to get BGR values
3. Start with tolerance of [30, 30, 30] and adjust as needed
4. Watch the binary mask in RViz to verify detection

**Common tape colors (BGR):**
- Blue tape: `[255, 50, 50]`
- Red tape: `[50, 50, 255]`
- Yellow tape: `[50, 255, 255]`
- Green tape: `[50, 255, 50]`
- Black tape: `[30, 30, 30]`

## Troubleshooting

### No tape detected
- Check tape color values match your actual tape
- Increase `color_tolerance`
- Decrease `min_contour_area`
- View `/tape_debug/image` in RViz to see binary mask

### Poor real-time performance
- Run with sudo for SCHED_FIFO scheduling
- Reduce image resolution (already at optimal 320×240)
- Increase `deadline_ms` if needed
- Close other CPU-intensive processes

### Erratic steering
- Adjust `kp_lateral` and `kp_angle` weights
- Increase `min_contour_area` to filter noise
- Check lighting conditions (affects color detection)

## Safety Notes

⚠️ **Motor control is disabled by default** for testing safety. Enable only when:
- Tape detection is working reliably
- Rover is in a safe testing environment
- Emergency stop is readily accessible

## See Also
- `FIFO_Hough.c` - C++ template for real-time image processing
- `line_follow_amy.py` - Production line following node
