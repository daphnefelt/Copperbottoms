
import pyrealsense2 as rs
import numpy as np
import cv2

# Start RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()

# Enable color stream
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

pipeline.start(config)

try:
    while True:
    # Wait for a frame
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()

    if not color_frame:
        continue

# Convert to numpy array
    image = np.asanyarray(color_frame.get_data())

# Get center pixel (for easy reading)
    h, w, _ = image.shape
    center_pixel = image[h // 2, w // 2]

# Separate BGR values
    blue = center_pixel[0]
    green = center_pixel[1]
    red = center_pixel[2]

    print(f"B: {blue} | G: {green} | R: {red}")

# Show image with marker
    cv2.circle(image, (w // 2, h // 2), 5, (0, 255, 255), -1)
    cv2.imshow("RealSense Color Stream", image)

# Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
