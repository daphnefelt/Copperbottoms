# where is blue 

import pyrealsense2 as rs
import numpy as np

# ---------------- START REALSENSE ----------------
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            continue

        image = np.asanyarray(color_frame.get_data())
        height, width, _ = image.shape

        # ---------------- BLUE MASK ----------------
        # OpenCV/RealSense uses BGR order
        blue_mask = (
            (image[:, :, 0] > 100) &     # Blue high
            (image[:, :, 1] > 100) &     # Green medium
            (image[:, :, 1] < 150) &
            (image[:, :, 2] < 100)       # Red low
        )

        # ---------------- FIND BLUE PIXELS ----------------
        y_pixels, x_pixels = np.where(blue_mask)

        if len(x_pixels) == 0:
            print("No blue detected")
            continue

        # ---------------- CENTER ESTIMATES ----------------
        mean_x = np.mean(x_pixels)
        median_x = np.median(x_pixels)

        # normalize from -1 to +1
        normalized_x = (mean_x - width / 2) / (width / 2)

        # ---------------- PRINT RESULTS ----------------
        print("Blue detected")
        print(f"Pixel count : {len(x_pixels)}")
        print(f"Mean X      : {mean_x:.2f}")
        print(f"Median X    : {median_x:.2f}")
        print(f"NormalizedX : {normalized_x:.3f}")

        # helpful direction labels
        if mean_x < width * 0.4:
            print("Object is LEFT")
        elif mean_x > width * 0.6:
            print("Object is RIGHT")
        else:
            print("Object is CENTER")

        print("-" * 40)

finally:
    pipeline.stop()