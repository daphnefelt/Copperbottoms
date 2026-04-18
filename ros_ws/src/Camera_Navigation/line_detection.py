# take and image and detect blue lines in it
import cv2
import numpy as np

img = cv2.imread('../img.jpg')
# blue mask
rgb = [26, 132, 194]
range_rgb = [10,10]
mask = cv2.inRange(img, (100, 0, 0), (255, 50, 50))
# find edges
edges = cv2.Canny(mask, 50, 150, apertureSize=3)
# find lines
lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=50, minLineLength=50, maxLineGap=10)
# draw lines
if lines is not None:
    for line in lines:
        x1, y1, x2, y2 = line[0]
        cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
cv2.imshow('lines', img)