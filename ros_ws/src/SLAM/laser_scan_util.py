from sensor_msgs.msg import LaserScan
import numpy as np



def get_angles_rad(msg: LaserScan):
    return msg.angle_min + msg.angle_increment*np.arange(0, len(msg.ranges))


def cone_indices(msg: LaserScan, center_rad: float, half_cone_rad: float):
    start_idx = int(((center_rad - half_cone_rad) - msg.angle_min) / msg.angle_increment)
    end_idx   = int(((center_rad + half_cone_rad) - msg.angle_min) / msg.angle_increment)
    n = len(msg.ranges)
    start_idx = max(0, min(start_idx, n - 1))
    end_idx   = max(0, min(end_idx,   n - 1))
    return start_idx, end_idx


def get_front(msg: LaserScan, half_cone_rad, angles=None):
    start_idx, end_idx =  cone_indices(msg, 0, half_cone_rad)
    return msg.ranges[start_idx: max(end_idx+1, len(msg.ranges))]
    
    
