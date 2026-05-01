import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pymavlink import mavutil
import time

class CmdVelToMavlink(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_mavlink')

        conn_str = '/dev/ttyACM0'
        baud = 115200
        self.max_throttle = 1 # m/s
        self.max_steering = 1 # rad/s

        # connect
        self.get_logger().info(f'CONNECTING')
        self.mav = mavutil.mavlink_connection(conn_str, baud=baud)
        self.mav.wait_heartbeat()
        self.get_logger().info('CONNECTED')

        # set to manual mode and arm
        self._set_mode('MANUAL')
        self._arm()

        # start listening to /cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

    # velocity callback
    def cmd_vel_callback(self, msg: Twist):
        # Normalize to [-1000, 1000] for RC override
        throttle = self._clamp(msg.linear.x  / self.max_throttle, -1.0, 1.0)
        steering  = self._clamp(msg.angular.z / self.max_steering, -1.0, 1.0)

        throttle_pwm = self._to_pwm(throttle)
        steering_pwm  = self._to_pwm(steering)

        self._send_rc_override(throttle_pwm, steering_pwm)

    def _send_rc_override(self, throttle_pwm: int, steering_pwm: int):
        # ch1 = steering
        # ch3 = throttle
        self.mav.mav.rc_channels_override_send(
            self.mav.target_system,
            self.mav.target_component,
            steering_pwm, # ch1
            0,
            throttle_pwm, # ch 3
            0,
            0, 0, 0, 0
        )
        self.get_logger().debug(f'Steering PWM: {steering_pwm}, Throttle PWM: {throttle_pwm}')

    def _set_mode(self, mode_name: str):
        mode_id = self.mav.mode_mapping().get(mode_name)
        if mode_id is None:
            self.get_logger().warn(f'Mode {mode_name} not found')
            return
        self.mav.mav.set_mode_send(
            self.mav.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        self.get_logger().info(f'Mode set to {mode_name}')

    def _arm(self):
        self.mav.arducopter_arm()
        self.mav.motors_armed_wait()
        self.get_logger().info('ARMED')

    @staticmethod
    def _to_pwm(normalized: float) -> int:
        # 1500 is neutral, 1000 is full reverse/left, 2000 is full forward/right
        return int(1500 + normalized * 500)

    @staticmethod
    def _clamp(value, low, high):
        return max(low, min(high, value))

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToMavlink()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._send_rc_override(1500, 1500)  # neutral on shutdown
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()