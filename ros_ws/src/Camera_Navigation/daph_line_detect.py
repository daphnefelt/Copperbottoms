# take and image and detect blue lines in it
import cv2
import numpy as np
import os
img = cv2.imread(os.path.join(os.path.dirname(__file__), '../img4.jpg'))

# WARP
h, w = img.shape[:2]

top_shift = 40   # shift top of image right (positive) or left (negative)
bottom_shift = -225 # shift bottom of image right (positive) or left (negative)

src = np.float32([
    [0, 0],           # top-left
    [w - top_shift, 0],       # top-right
    [0, h],        # bottom-left
    [w - bottom_shift, h]     # bottom-right
])

dst = np.float32([
    [0, 0],
    [w, 0],
    [0, h],
    [w, h]
])

M = cv2.getPerspectiveTransform(src, dst)
img = cv2.warpPerspective(img, M, (w, h))

# blue mask
rgb = [164, 108, 7]
plus_minus = [50, 50, 90]
mask = cv2.inRange(img, np.array(rgb) - np.array(plus_minus), np.array(rgb) + np.array(plus_minus))
# find edges
edges = cv2.Canny(mask, 50, 150, apertureSize=3)
# find lines
lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=50, minLineLength=50, maxLineGap=10)
# draw lines
if lines is not None:
    for line in lines:
        x1, y1, x2, y2 = line[0]
        cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 2)

import matplotlib.pyplot as plt

img_width = img.shape[1]
img_center = img_width / 2
third = img_width / 3
third_left = third
third_right = third_left + third

# draw lines and find highest point
min_y = img.shape[0]
target_x = None
go_straight = True
if lines is not None:
    for line in lines:
        x1, y1, x2, y2 = line[0]
        cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        if y1 < min_y:
            min_y = y1
            target_x = x1
        if y2 < min_y:
            min_y = y2
            target_x = x2

        # if not in middle third, set go_straight to False
        if (x1 < third_left or x1 > third_right) or (x2 < third_left or x2 > third_right):
            go_straight = False

# To show the mask:
plt.subplot(1, 2, 1)
plt.imshow(mask, cmap='gray')
plt.title('Mask')

# To show the original with lines drawn:
plt.subplot(1, 2, 2)
plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))  # convert BGR->RGB for correct colors
plt.title('Lines')

# add the min y point
if not go_straight:
    plt.plot(target_x, min_y, 'ro', markersize=12, label='Target Point')
plt.axvline(third_left, color='yellow', linestyle='--', linewidth=1, label='Left/Mid Boundary')
plt.axvline(third_right, color='yellow', linestyle='--', linewidth=1, label='Mid/Right Boundary')
plt.legend()

plt.show()