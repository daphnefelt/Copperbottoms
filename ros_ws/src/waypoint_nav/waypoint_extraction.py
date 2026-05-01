import math
import os

# Always finds the file relative to where this script is
script_dir = os.path.dirname(os.path.abspath(__file__))
pose_file = os.path.join(script_dir, 'pose_history_save.txt')

# Load poses
poses = []
with open(pose_file, 'r') as f:
    for line in f:
        x, y, yaw = map(float, line.strip().split())
        poses.append((x, y, yaw))

# Extract every Nth pose
N = 10
waypoints = poses[::N]

print(f"Total poses: {len(poses)}, Waypoints extracted: {len(waypoints)}")
for i, (x, y, yaw) in enumerate(waypoints):
    print(f"  WP{i+1}: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}")