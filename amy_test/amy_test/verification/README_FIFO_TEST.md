# FIFO RealSense Line Detection Performance Test

## Purpose

Measure real-time performance of Hough line detection on RealSense camera to determine optimal resolution and validate timing requirements for line following control loop.

## Files

- `fifo_realsense_linedetect.c` - SCHED_FIFO test for RealSense camera with Hough lines
- `FIFO_Hough.c` - Original multi-transform test (reference)
- `Makefile` - Build configuration
- `analyze_fifo_results.py` - CSV analysis and recommendation tool

## Requirements

```bash
# Install dependencies (if not already installed)
sudo apt-get install librealsense2-dev libopencv-dev

# Verify RealSense camera is connected
rs-enumerate-devices
```

## Build

```bash
cd ~/code/Copperbottoms/ros_ws/src/amy_test/amy_test/verification
make
```

## Run Test

```bash
# Run with SCHED_FIFO scheduler (requires sudo)
sudo ./fifo_realsense_linedetect

# Or use make target
make test
```

The test will:
1. Test Hough line detection at 3 resolutions (160×120, 320×240, 640×480)
2. Collect 100 frames per resolution
3. Measure execution time with microsecond precision
4. Generate CSV files with timing data

**During test:**
- Press 'q' to skip to next resolution
- Window shows live line detection results
- Terminal displays progress

## Analyze Results

```bash
python3 analyze_fifo_results.py
```

This generates a comparison report showing:
- Mean/StdDev/Max execution times
- Deadline miss percentages
- Recommendations for resolution selection
- Next steps for ROS integration

## Interpret Results

### Key Metrics

- **Mean exec time**: Average processing time per frame
  - Target: <50ms for 20 Hz control loop
  - Lower is better

- **Miss %**: Percentage of frames exceeding 50ms deadline
  - Target: <5% for good real-time performance
  - <1% for excellent performance

- **Max exec time**: Worst-case processing time
  - Important for understanding jitter
  - Should be <70ms to avoid control instabilities

### Expected Performance

| Resolution | Expected Mean | Use Case |
|------------|---------------|----------|
| 160×120    | 10-15ms       | Maximum responsiveness, basic line detection |
| 320×240    | 20-30ms       | Balanced performance/quality (recommended) |
| 640×480    | 40-60ms       | Best quality, marginal real-time performance |

### Decision Criteria

**Use Hough lines if:**
- Selected resolution has mean exec time <30ms
- Line detection accuracy is critical
- Willing to accept 2-3x overhead vs simple edge detection

**Stick with current Canny approach if:**
- All resolutions exceed 40ms mean exec time
- Speed is more important than accuracy
- Current system already meets tracking requirements

## Example Output

```
Resolution    Mean (ms)  StdDev (ms)    Max (ms)     Miss %
----------------------------------------------------------------------
160x120           12.34        2.11       18.45       0.00%
320x240           25.67        3.22       35.12       2.10%
640x480           48.92        5.67       62.34       8.45%

RECOMMENDATIONS FOR LINE FOLLOWING
Target: Mean exec time <50ms, Miss % <5%

160x120      ✓ EXCELLENT  (Mean:  12.3ms, Miss:  0.0%)
320x240      ✓ EXCELLENT  (Mean:  25.7ms, Miss:  2.1%)
640x480      ⚠ MARGINAL   (Mean:  48.9ms, Miss:  8.5%)

RECOMMENDATION: Use 320x240 for best quality among suitable options
```

## Integration with ROS

After selecting optimal resolution:

1. **Update camera publisher** (if needed):
   ```bash
   # Edit ros_stream_with_depth.py to match selected resolution
   # Or configure via ROS parameters
   ```

2. **Proceed to Phase 1-4**: ROS monitoring
   - Run `ros2 launch amy_test latency_monitoring.launch.py`
   - Measure end-to-end camera→motor latency
   - Validate total latency <100ms

3. **Compare FIFO vs ROS latency**:
   - FIFO exec time: Vision processing only (best case)
   - ROS latency: Vision + message overhead + Python processing
   - Expected ROS overhead: +10-20ms

## Troubleshooting

**Error: "sched_setscheduler failed"**
- Must run with sudo: `sudo ./fifo_realsense_linedetect`
- SCHED_FIFO requires root privileges

**Error: "could not open camera"**
- Check RealSense connection: `rs-enumerate-devices`
- Ensure no other process using camera (close RealSense Viewer)

**High deadline miss %**
- Normal for 640×480 resolution
- Try lower resolution (320×240 or 160×120)
- Check CPU load: `top` (close other applications)

**Compilation errors**
- Verify librealsense2 installed: `pkg-config --modversion realsense2`
- Verify OpenCV installed: `pkg-config --modversion opencv`

## CSV File Format

Generated files: `fifo_realsense_houghlines_<resolution>.csv`

```csv
frame,start_us,end_us,exec_us,delta_us,deadline_us,miss,lines_detected
0,1234567890,1234578901,11011,0,50000,0,12
1,1234589012,1234599123,10111,20111,50000,0,15
...
# Summary (excluding frame 0)
# Frames analyzed,99
# Average exec time us,12345.678
# Exec std dev us,2100.123
...
```

## Next Steps

1. Run FIFO test to establish baseline
2. Analyze results with Python script
3. Document findings in verification/readme
4. Proceed to Phase 1: Implement ROS monitoring nodes
5. Compare ROS measurements with FIFO baseline

---

For questions or issues, refer to the main project documentation in `amy_test/verification/readme`.
