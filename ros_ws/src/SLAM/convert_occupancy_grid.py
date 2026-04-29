import numpy as np
from PIL import Image
import yaml

# Load the binary occupancy grid
grid = np.load("last_lidar_grid.npy")  # shape: (800, 800), values: 0 (free), 100 (occupied)

# Flip vertically for ROS map convention (origin at bottom left)
grid = np.flipud(grid)

# Convert to PGM image (0=free/white, 100=occupied/black, 205=unknown/gray)
img = np.full(grid.shape, 205, dtype=np.uint8)  # unknown by default
img[grid == 0] = 254  # free (almost white)
img[grid == 100] = 0  # occupied (black)

im = Image.fromarray(img)
im.save("map.pgm")

# Write the YAML file
map_metadata = {
    'image': 'map.pgm',
    'resolution': 0.05,  # meters per pixel (match your LIDAR_GRID_RES)
    'origin': [20.0, 20.0, 0.0],  # [x, y, yaw]
    'negate': 0,
    'occupied_thresh': 0.65,
    'free_thresh': 0.196,
    'mode': 'trinary'
}

with open('map.yaml', 'w') as f:
    yaml.dump(map_metadata, f, default_flow_style=False)

print("Map image and YAML saved as map.pgm and map.yaml")