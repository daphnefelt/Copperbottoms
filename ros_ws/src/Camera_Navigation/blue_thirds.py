import pyrealsense2 as rs
import numpy as np

# Start RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            continue

        image = np.asanyarray(color_frame.get_data())
        height, width, _ = image.shape

        # Split camera frame into thirds
        Left_third = image[:, :width//3]
        Middle_third = image[:, width//3:2*width//3]
        Right_third = image[:, 2*width//3:]

        right = []
        middle = []

        # Double for loop over all pixels
        for i in range(height):
            for j in range(width):
                b, g, r = image[i, j]
                print(f"Pixel ({i}, {j}): R={r}, G={g}, B={b}")
                if b>100 and 100<g<150 and r<100:
                    # Right third
                    if j >= 2*width//3:
                        print("Blue detected Right")
                        right.append(1)
                    # Middle third
                    elif j >= width//3:
                        print("Blue detected Middle")
                        middle.append(1)
        if len(right) > len(middle):
            print("Blue object is MOSTLY right")
        elif len(middle) > len(right):
            print("Blue object is MOSTLY middle")
        else:            
            print("Blue object is equally in middle and right")
        

finally:
    pipeline.stop()
