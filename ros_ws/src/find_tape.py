#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
import cv2 as cv
from std_msgs.msg import Float64
from std_msgs.msg import Bool


# Discrete angle outputs (radians) — tune these to your robot's response
ANGLE_STRAIGHT = 0.0
ANGLE_GENTLE   = 0.3
ANGLE_HARD     = 0.6

# How long to hold last known angle before giving up (seconds)
NO_TAPE_TIMEOUT = 1.0


class FindTapeNode(Node):
    def __init__(self) -> None:
        super().__init__("find_tape")
        self.color_topic = "/camera/color/image_raw"
        self.depth_topic = "/camera/depth/image_raw"

        # H=100 ±3, S and V fully open
        self.lower_hsv = np.array([97,   0,   0])
        self.upper_hsv = np.array([103, 255, 255])

        self.latest_depth = None
        self.latest_depth_stamp = None
        self.frame_count = 0
        self.processes_every_n_frames = 1
        self.last_known_angle = ANGLE_STRAIGHT

        self.tape_lost_since = None   # timestamp when tape was last seen to be gone

        self.color_sub = self.create_subscription(
            Image, self.color_topic, self.color_callback, qos_profile_sensor_data)
        self.depth_sub = self.create_subscription(
            Image, self.depth_topic, self.depth_callback, qos_profile_sensor_data)
        self.angle_pub = self.create_publisher(Float64, "/angle_goal", 10)
        self.stop_pub  = self.create_publisher(Bool,    "/tape_stop",  10)

        self.get_logger().info(f"Subscribed color: {self.color_topic}")
        self.get_logger().info(f"Subscribed depth: {self.depth_topic}")
        self.get_logger().info(f"HSV range: lower={self.lower_hsv} upper={self.upper_hsv}")
        self.get_logger().info(f"No-tape timeout: {NO_TAPE_TIMEOUT}s")

    @staticmethod
    def image_to_bgr(msg: Image) -> np.ndarray:
        if msg.encoding.lower() != "bgr8":
            raise ValueError(f"Expected bgr8, got {msg.encoding}")
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        return arr.reshape((msg.height, msg.width, 3))

    @staticmethod
    def image_to_depth_mm(msg: Image) -> np.ndarray:
        if msg.encoding.upper() != "16UC1":
            raise ValueError(f"Expected 16UC1, got {msg.encoding}")
        arr = np.frombuffer(msg.data, dtype=np.uint16)
        return arr.reshape((msg.height, msg.width))

    def depth_callback(self, msg: Image) -> None:
        try:
            self.latest_depth = self.image_to_depth_mm(msg)
            self.latest_depth_stamp = msg.header.stamp
        except Exception as exc:
            self.get_logger().error(f"Depth parse error: {exc}")

    def color_callback(self, msg: Image) -> None:
        self.frame_count += 1
        if self.frame_count % self.processes_every_n_frames != 0:
            return
        if self.latest_depth is None:
            self.get_logger().warn("No depth frame received yet.")
            return
        try:
            color = self.image_to_bgr(msg)
        except Exception as exc:
            self.get_logger().error(f"Color parse error: {exc}")
            return

        h, w = color.shape[:2]

        # --- HSV mask ---
        hsv = cv.cvtColor(color, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv, self.lower_hsv, self.upper_hsv)

        # --- Split into vertical thirds ---
        third = w // 3
        left_tape   = np.any(mask[:, :third]        > 0)
        center_tape = np.any(mask[:, third:2*third] > 0)
        right_tape  = np.any(mask[:, 2*third:]      > 0)

        tape_visible = left_tape or center_tape or right_tape
        now = self.get_clock().now().nanoseconds * 1e-9  # seconds

        # --- Timeout logic ---
        if tape_visible:
            self.tape_lost_since = None   # reset timer whenever tape is seen
            should_stop = False
        else:
            if self.tape_lost_since is None:
                self.tape_lost_since = now  # start the clock
            elapsed = now - self.tape_lost_since
            should_stop = elapsed >= NO_TAPE_TIMEOUT

        # --- Publish stop signal ---
        stop_msg = Bool()
        stop_msg.data = should_stop
        self.stop_pub.publish(stop_msg)

        if should_stop:
            self.get_logger().warn(
                f"Tape lost for >{NO_TAPE_TIMEOUT}s — publishing stop"
            )
            cv.imshow("find_tape", color)
            cv.waitKey(1)
            return  # don't publish an angle_goal; pid node will stop

        # --- Decision table ---
        if not tape_visible:
            angle_goal = self.last_known_angle
            state = "NO TAPE (in window) — holding last"

        elif center_tape and not left_tape and not right_tape:
            angle_goal = ANGLE_STRAIGHT
            state = "CENTER only  → straight"

        elif center_tape and right_tape and not left_tape:
            angle_goal = -ANGLE_GENTLE
            state = "CENTER+RIGHT → gentle right"

        elif center_tape and left_tape and not right_tape:
            angle_goal = ANGLE_GENTLE
            state = "CENTER+LEFT  → gentle left"

        elif right_tape and not center_tape and not left_tape:
            angle_goal = -ANGLE_HARD
            state = "RIGHT only   → hard right"

        elif left_tape and not center_tape and not right_tape:
            angle_goal = ANGLE_HARD
            state = "LEFT only    → hard left"

        else:
            angle_goal = ANGLE_STRAIGHT
            state = "ALL thirds   → straight"

        self.last_known_angle = angle_goal

        # --- Debug overlay ---
        display = color.copy()
        cv.line(display, (third,   0), (third,   h), (255, 255, 0), 1)
        cv.line(display, (2*third, 0), (2*third, h), (255, 255, 0), 1)
        tape_overlay = np.zeros_like(display)
        if left_tape:
            tape_overlay[:, :third]        = [0, 0, 80]
        if center_tape:
            tape_overlay[:, third:2*third] = [0, 80, 0]
        if right_tape:
            tape_overlay[:, 2*third:]      = [80, 0, 0]
        display = cv.addWeighted(display, 1.0, tape_overlay, 0.4, 0)
        cv.putText(display, state, (10, 30),
                   cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv.putText(display, f"angle_goal: {angle_goal:.2f} rad", (10, 60),
                   cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        if self.tape_lost_since is not None:
            elapsed = now - self.tape_lost_since
            cv.putText(display, f"tape lost: {elapsed:.1f}s / {NO_TAPE_TIMEOUT}s",
                       (10, 90), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 100, 255), 2)
        cv.imshow("find_tape", display)
        cv.waitKey(1)

        # --- Publish angle ---
        angle_msg = Float64()
        angle_msg.data = angle_goal
        self.angle_pub.publish(angle_msg)
        self.get_logger().info(
            f"Frame {self.frame_count} | {state} | angle_goal={angle_goal:.2f} rad"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FindTapeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
