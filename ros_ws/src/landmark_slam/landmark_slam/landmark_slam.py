#!/usr/bin/env python3
import rclpy
import numpy as np
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from custom_messages.msg import Slow
import numpy as np





class LandmarkSLAM(Node):
	def __init__(self):
		self.state = np.zeros(3)
		pass
	


def main():
    print('Hi from landmark_slam.')


if __name__ == '__main__':
    main()
