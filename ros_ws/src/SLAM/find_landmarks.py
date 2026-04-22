import cv2
import numpy as np
import math
from pupil_apriltags import Detector

# RealSense D435 640x480 defaults
_FX = 615.0 # focal length x (pixels)
_CX = 320.0 # principal point x (pixels)

_detector = Detector(families='tag36h11')

def detect_landmark(img, depth, fx=_FX, cx=_CX):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img

    detections = _detector.detect(gray)
    results = []

    for tag in detections:
        tag_cx = tag.center[0]  # pixel x of tag center
        tag_cy = tag.center[1]  # pixel y of tag center

        # Clamp to depth image bounds
        h, w = depth.shape[:2]
        x_px = int(max(0, min(round(tag_cx), w - 1)))
        y_px = int(max(0, min(round(tag_cy), h - 1)))

        depth_m = float(depth[y_px, x_px]) / 1000.0   # mm to meters (to match lidar data)

        # angle from robot forward axis
        angle = math.atan2(tag_cx - cx, fx) # radians, + = right

        results.append({
            'id': int(tag.tag_id),
            'depth': depth_m,
            'angle': angle,
        })

    return results