# Advanced Tape Loss Recovery System

## Overview
The tape contour detection node now has a sophisticated **5-phase recovery system** designed specifically for handling the **4 sharp right turns** in your track.

## Recovery Phases

### Phase 1: **COASTING** (5 frames / ~170ms)
**Status Display:** Yellow "COASTING"  
**Behavior:**
- Maintains last steering command at 80% strength
- Reduces forward speed to 70%
- Handles brief tape occlusions or noise

### Phase 2: **SEARCHING** (20 frames / ~700ms)
**Status Display:** Orange "SEARCHING"  
**Behavior:**
- Stops forward motion
- Turns in place to find tape
- **Sharp turn detection:** If sharp turn detected, uses faster turn rate (0.8 rad/s)
- **Right turn bias:** For sharp turns, biases toward right turns (track has 4 right turns)
- Normal search: turns toward side where tape was last seen

### Phase 3a: **BACKING_UP** (3 seconds or until tape found)
**Status Display:** Orange "BACKING_UP (X.Xs/3.0s)"  
**Behavior:**
- Reverses at 70% of forward speed
- Backs up straight (no turning)
- Gives rover space to reacquire tape
- **Early exit:** If tape found during backup, immediately returns to tracking

### Phase 3b: **TURN_RIGHT** (30 degrees or until tape found)
**Status Display:** Orange "TURN_RIGHT (X.X°/30.0°)"  
**Behavior:**
- Stops forward motion
- Turns right at 0.4 rad/s
- Searches for tape while turning
- Accumulates turn angle
- **Early exit:** If tape found, immediately returns to tracking

### Phase 3c: **TURN_LEFT** (60 degrees total or until tape found)
**Status Display:** Orange "TURN_LEFT (X.X°/60.0°)"  
**Behavior:**
- First returns to center (undoes 30° right turn)
- Then turns left 30° from center
- Searches for tape throughout
- **Early exit:** If tape found, immediately returns to tracking

### Phase 3d: **STOPPED** (final state)
**Status Display:** Red "STOPPED"  
**Behavior:**
- All recovery attempts exhausted
- Stops completely (safety)
- Logs error message
- Waits for manual intervention

## Sharp Turn Detection

The system automatically detects sharp turns by monitoring tape angle changes:
- **Threshold:** 45° angle change between frames
- **Action:** Triggers more aggressive search behavior
- **Right turn bias:** Since track has 4 right turns, sharp turns bias toward right search

### Sharp Turn Search Strategy:
```python
if sharp_turn_detected:
    search_turn = 0.8 rad/s  # Faster than normal 0.5 rad/s
    if tape_was_on_right:
        search_turn = 0.8   # Aggressive right turn
    else:
        search_turn = -0.6  # Less aggressive left
```

## Visual Feedback (RViz 2)

### Status Colors:
- **Green** - TRACKING (following tape normally)
- **Yellow** - COASTING (brief loss, coasting)
- **Orange** - SEARCHING / BACKING_UP / TURN_RIGHT / TURN_LEFT (active recovery)
- **Red** - STOPPED (gave up)

### Status Display Shows:
- Current phase name
- Progress indicator (time or angle remaining)
- Example: `BACKING_UP (1.2s/3.0s)` or `TURN_RIGHT (15.3°/30.0°)`

## Recovery Flow Diagram

```
Tape Lost
    │
    ▼
COASTING (5 frames)
    │
    ├─► Tape found? → Return to TRACKING
    │
    ▼
SEARCHING (20 frames)
    │
    ├─► Tape found? → Return to TRACKING
    │   [Sharp turn detected? → More aggressive search]
    │
    ▼
BACKING_UP (3 seconds)
    │
    ├─► Tape found? → Return to TRACKING
    │
    ▼
TURN_RIGHT (30°)
    │
    ├─► Tape found? → Return to TRACKING
    │
    ▼
TURN_LEFT (60° total)
    │
    ├─► Tape found? → Return to TRACKING
    │
    ▼
STOPPED (give up)
```

## Configuration Parameters

Located in [tape_contour_rt.py](tape_contour_rt.py):

```python
# Coast phase
self.max_coast_frames = 5          # ~170ms @ 30Hz

# Search phase  
self.max_search_frames = 20        # ~700ms @ 30Hz

# Backup phase
self.backup_duration = 3.0         # 3 seconds

# Turn search phases
self.turn_search_angle = 30.0      # 30 degrees each direction

# Sharp turn detection
self.angle_change_threshold = 45.0 # degrees
```

## Tuning for Your Track

### If missing sharp turns frequently:
1. **Increase sharp turn threshold:**
   ```python
   self.angle_change_threshold = 35.0  # More sensitive
   ```

2. **Increase right turn search speed:**
   ```python
   search_turn = 1.0  # Was 0.8 rad/s
   ```

3. **Extend search time:**
   ```python
   self.max_search_frames = 30  # Was 20
   ```

### If backing up too much:
1. **Reduce backup duration:**
   ```python
   self.backup_duration = 2.0  # Was 3.0 seconds
   ```

2. **Skip backup phase** (if not needed):
   - Comment out BACKING_UP phase in code
   - Go directly from SEARCHING to TURN_RIGHT

### If turning too far during recovery:
1. **Reduce turn search angle:**
   ```python
   self.turn_search_angle = 20.0  # Was 30.0 degrees
   ```

## Testing Tips

1. **Watch the status in RViz 2** - The color-coded status shows exactly what phase the rover is in

2. **Check the logs** - Recovery phase transitions are logged:
   ```
   [WARN] Tape lost - coasting (frame 1/5)
   [WARN] Entering search mode - turning to find tape
   [WARN] Starting advanced recovery: BACKING UP
   [WARN] Backup complete. Starting RIGHT turn search
   [INFO] Tape reacquired after 45 frames (Recovery phase: TURN_RIGHT)
   ```

3. **Monitor sharp turn detection**:
   ```
   [INFO] Sharp turn detected! Angle change: 52.3°
   ```

4. **Test each sharp turn** - Verify rover successfully navigates all 4 right turns

## Launch Command

```bash
source ~/code/Copperbottoms/ros_ws/install/setup.bash

# With motor control enabled
ros2 launch amy_test tape_contour_rt.launch.py enable_motor_control:=true

# View in RViz 2
rviz2
# Add topic: /tape_debug/image
```

## Expected Behavior on Sharp Turns

1. Rover approaches sharp right turn
2. **Sharp turn detected** (angle change > 45°)
3. Tape temporarily lost as rover overshoots
4. **COASTING:** Maintains last turn command
5. **SEARCHING:** Aggressive right turn (0.8 rad/s) with right bias
6. Tape reacquired, returns to TRACKING
7. Successfully navigates turn

If this fails:
- **BACKING_UP:** Reverses to get better view
- **TURN_RIGHT:** Systematically searches right 30°
- **TURN_LEFT:** Returns to center, searches left 30°
- **STOPPED:** Gives up (should rarely happen)

## Success Metrics

Monitor in logs every 10 frames:
```
Frame 100: 12.3ms | Tape: True | Turn: 0.45 | Error: 0.12 [P=0.32 I=0.05 D=0.02]
```

- **Tape: True** - Successfully tracking
- **Recovery phases** - Should be brief (< 3 seconds)
- **Sharp turns** - Should auto-detect and handle
