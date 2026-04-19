import pyrealsense2 as rs
import numpy as np

# Start RealSense pipeline
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

        # Split indices
        left_end = width // 3
        mid_end = 2 * width // 3

        # Counters
        left_count = 0
        middle_count = 0
        right_count = 0

        # Scan image (no printing per pixel!)
        for i in range(height):
            for j in range(width):
                b, g, r = image[i, j]

                # Simple blue threshold
                if b > 120 and g < 100 and r < 100:
                    if j < left_end:
                        left_count += 1
                    elif j < mid_end:
                        middle_count += 1
                    else:
                        right_count += 1

        # Decide dominant region
        if left_count > middle_count and left_count > right_count:
            print("Blue is MOSTLY LEFT")
        elif middle_count > left_count and middle_count > right_count:
            print("Blue is MOSTLY MIDDLE")
        elif right_count > left_count and right_count > middle_count:
            print("Blue is MOSTLY RIGHT")
        else:
            print("Blue is split or not clearly dominant")

finally:
    pipeline.stop()
