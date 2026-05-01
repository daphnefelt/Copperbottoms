from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from std_msgs.msg import String
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from std_msgs.msg import String
from enum import Enum

class HALLWAY_STATE(Enum):
    HALLWAY_ONE = 1
    HALLWAY_TWO = 2
    HALLWAY_THREE = 3
    HALLWAY_FOUR = 4

class DRIVE_ORIENT(Enum):
    FORWARD =1
    REVERSE = 2



# MARY fix these values after ekf mapping with Daphne or with lidar
HALLWAY1_YMAX =14
HALLWAY2_XMAX = 50

HALLWAY3_YMIN = -10
HALLWAY4_XMIN = -2


class MotionPlanner(Node):

    def __init__(self, normal_drive_orientation=DRIVE_ORIENT.FORWARD, map_points=[], global_waypoints=[]):
        super().__init__('motion_planner')
        self.mode = "STRAIGHT"

        self.normal_drive_orientation= normal_drive_orientation

        self.planned_trip = True 
        self.map_points = map_points
        self.global_waypoints = global_waypoints

        if not self.map_points or not self.global_waypoints:
            self.planned_trip = False

        # x, y, quarternion - for heading
        self.pos = np.zeros(3)

        self.hallway_loc = HALLWAY_STATE.HALLWAY_ONE

        # make a decision - lidar is 5.5Hz every .18 seconds
        self.create_timer(0.2, self.loop)



        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        
        self.pose_pub = self.create_subscription(PoseWithCovarianceStamped, '/slam/pose', self._update_state, 10)
        self.map_pub = self.create_subscription(String, '/slam/landmarks', 10)


    def _set_pos(self, x, y, z):
        self.pos = np.array([x, y, z])


    def _enter_mode(self,mode):
        self.mode = mode

    def _set_current_hallway(self):
        x_m, y_m, z_rad = self.pos
        # always define corner spaces to be part of the next hallway
        if x_m < 2 and x_m > -2 and y_m < HALLWAY1_YMAX:
            self.hallway_loc = HALLWAY_STATE.HALLWAY_ONE

        elif y_m > HALLWAY1_YMAX and x_m < HALLWAY2_XMAX:
            self.hallway_loc = HALLWAY_STATE.HALLWAY_TWO

        elif x_m > HALLWAY2_XMAX and y_m > HALLWAY3_YMIN:
            self.hallway_loc = HALLWAY_STATE.HALLWAY_THREE
        else:
            self.hallway_loc = HALLWAY_STATE.HALLWAY_FOUR





        """Determine what mode should be based on assumed true estimate of state
        """
    def _determine_mode(self):
        pass

    def get_valid_laser_scan(self, msg: LaserScan):
        # corresponds to ~20ft
        MAX_DIST_m = 6
        ranges = np.array(msg.ranges)
        ranges = np.where(np.isinf(ranges), 99.0, ranges)
        angles = msg.angle_min  + msg.angle_increment* np.arange(0, len(ranges))
        angles = angles[ranges < MAX_DIST_m and ranges > msg.range_min]
        ranges = ranges[ranges < MAX_DIST_m and ranges > msg.range_min]
        return (angles, ranges)
    
        # 2. SVD fit + RANSAC-lite refit
        def _svd_angle(pts):
            _, _, vt = np.linalg.svd(pts - pts.mean(axis=0))
            return vt[0]

        dx, dy = _svd_angle(xy)
        cx, cy = xy.mean(axis=0)
        residuals = np.abs(dx * (xy[:, 1] - cy) - dy * (xy[:, 0] - cx))
        xy = xy[residuals < residual_thresh]
        if len(xy) < min_points:
            return float('nan')

        dx, dy = _svd_angle(xy)
        return float(np.arctan2(dy, dx))




    def generate_line_segments(self, msg: LaserScan):
        # RPLIDAR resolution 1% of range, so up to 6m = .06
        # RPLIDAR accuracy 1% of the range（≤3 m） 2% of the range（3-5 m）2.5% of the range（5-25m)
        # so max error at 6meters is .15
        MIN_POINTS_TO_LINE = 5
        MAX_RESIDUAL = .5
        # concerned with the max residual
        pass


    def scan_callback(self, msg: LaserScan):
        # detect corners and detect hallway narrowing

        # fit line and if the residuals are large on average we have an improperly fit line
        # throw out portions of the line and rebuild slowly to find changes in line
        pass

    def _update_state(self, msg: PoseWithCovarianceStamped):
        x, y, _ = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
        z = msg.pose.pose.orientation.w
        self._set_pos(x, y, z)
        self._set_current_hallway()
        self._enter_mode(self._determine_mode())


    def loop(self):
        # in absence of new information - go off old information and 
        # check for critical areas and danger areas
        
        if self.mode == "RECOVERY":
            # whether left or right, forward turn or backward turn
            pass





# -- Entry point --------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MotionPlanner()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.vel_pub.publish(Twist())
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
