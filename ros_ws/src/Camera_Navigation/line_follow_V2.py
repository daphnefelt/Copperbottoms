import rclpy
import numpy as np
from numpy.lib.stride_tricks import as_strided
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Bool


class LineFollowerV2(Node):

    def __init__(self):
        super().__init__('line_follower_v2')

        image_topic = '/camera/color/image_raw'
        cmd_vel_topic = '/cmd_vel'

        # parameters
        self.forward_speed = 0.22
        self.min_speed = 0.20
        self.search_speed = 0.20
        self.max_turn = 0.55
        self.search_turn = 0.50
        self.kp = 0.34
        self.kd = 0.18
        self.error_offset = -0.18  # camera-lens offset calibrated for robot-frame centering

        # blue-tape masking around target BGR
        self.target_bgr = np.array([204.0, 146.0, 39.0], dtype=np.float32)
        self.color_tolerance = np.array([62.0, 58.0, 55.0], dtype=np.float32)
        self.blue_score_threshold = 22.0
        self.min_track_pixels = 90

        # focus tracking mostly on lower image region
        self.roi_top_ratio = 0.55
        self.roi_bottom_ratio = 1.0

        # runtime state
        self.rover_armed = False
        self.seen_first_frame = False
        self.frame_count = 0
        self.last_error = 0.0
        self.last_turn = 0.0
        self.lost_frames = 0
        self.last_unbiased_error = 0.0
        self.last_biased_error = 0.0

        # subscribers
        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )
        self.armed_sub = self.create_subscription(
            Bool,
            '/rover/armed',
            self.armed_callback,
            10
        )

        # publishers
        self.vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        self.get_logger().info(
            f'Line follower node started. image_topic={image_topic}, cmd_vel_topic={cmd_vel_topic}'
        )



    """
    states
    """
    # follow
    # search
    # bridge


    
    
    """
    callbacks
    """
    def armed_callback(self, msg: Bool):
        was_armed = self.rover_armed
        self.rover_armed = bool(msg.data)

        if self.rover_armed and not was_armed:
            self.get_logger().info('Rover armed detected. Enabling line-follow motion commands.')
        elif (not self.rover_armed) and was_armed:
            self.get_logger().warn('Rover disarmed. Holding zero cmd_vel.')

    def image_callback(self, msg):
        self.frame_count += 1

        img = self.decode_image(msg)
        if img is None:
            return

        if not self.seen_first_frame:
            self.seen_first_frame = True
            self.get_logger().info(f"First camera frame received. encoding={msg.encoding}")

        if not self.rover_armed:
            self.publish_stop()
            return

        height, width, _ = img.shape
        y0 = int(height * self.roi_top_ratio)
        y1 = int(height * self.roi_bottom_ratio)
        roi = img[y0:y1, :, :]

        blue_mask = self.compute_blue_mask(roi)
        edge_map = self.canny_edge_detection(roi)
        edge_mask = edge_map > 0

        # Primary signal: blue edges. Fallback: full blue mask.
        track_mask = blue_mask & edge_mask
        track_pixels = int(np.sum(track_mask))
        if track_pixels < self.min_track_pixels:
            track_mask = blue_mask
            track_pixels = int(np.sum(track_mask))

        if track_pixels >= self.min_track_pixels:
            track_x = self.compute_track_center_x(track_mask)
            self.follow_track(track_x, width)

            if self.frame_count % 10 == 0:
                self.get_logger().info(
                    (
                        f"state=follow, blue_px={int(np.sum(blue_mask))}, edge_px={int(np.sum(edge_mask))}, "
                        f"track_px={track_pixels}, lin={self.forward_speed:.2f}, ang={self.last_turn:.2f}, "
                        f"err_unbiased={self.last_unbiased_error:.3f}, err_biased={self.last_biased_error:.3f}, "
                        f"offset={self.error_offset:.3f}"
                    )
                )
        else:
            self.search_for_track()


        




    """
    Helper functions
    """

    def publish_stop(self):
        cmd = Twist()
        self.vel_pub.publish(cmd)

    def follow_track(self, track_x: int, width: int):
        center_x = width / 2.0
        unbiased_error = (track_x - center_x) / max(center_x, 1.0)
        biased_error = unbiased_error + self.error_offset

        self.last_unbiased_error = float(unbiased_error)
        self.last_biased_error = float(biased_error)

        d_error = biased_error - self.last_error
        self.last_error = biased_error

        turn = -self.kp * biased_error - self.kd * d_error
        turn = float(np.clip(turn, -self.max_turn, self.max_turn))

        # Slow down as turn demand increases.
        turn_load = min(abs(turn) / max(self.max_turn, 1e-6), 1.0)
        speed = max(self.min_speed, self.forward_speed * (1.0 - 0.45 * turn_load))

        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = turn
        self.vel_pub.publish(cmd)

        self.last_turn = turn
        self.lost_frames = 0

    def search_for_track(self):
        self.lost_frames += 1
        spin_sign = -1.0 if self.last_error >= 0.0 else 1.0

        cmd = Twist()
        cmd.linear.x = self.search_speed
        cmd.angular.z = spin_sign * self.search_turn
        self.vel_pub.publish(cmd)

        if self.lost_frames % 10 == 0:
            self.get_logger().warn(
                (
                    f'No track detected: search mode, lost_frames={self.lost_frames}, '
                    f'ang={cmd.angular.z:.2f}, err_unbiased={self.last_unbiased_error:.3f}, '
                    f'err_biased={self.last_biased_error:.3f}, offset={self.error_offset:.3f}'
                )
            )

    def compute_track_center_x(self, mask: np.ndarray) -> int:
        ys, xs = np.where(mask)
        if xs.size == 0:
            return mask.shape[1] // 2
        return int(np.mean(xs))

    def compute_blue_mask(self, roi: np.ndarray) -> np.ndarray:
        roi_f = roi.astype(np.float32)
        channel_error = np.abs(roi_f - self.target_bgr)
        near_target = np.all(channel_error <= self.color_tolerance, axis=2)

        b = roi_f[:, :, 0]
        g = roi_f[:, :, 1]
        r = roi_f[:, :, 2]
        blue_score = b - 0.45 * (g + r)
        return near_target & (blue_score > self.blue_score_threshold)

    def decode_image(self, msg: Image):
        # Support common raw camera encodings and convert to BGR.
        enc = msg.encoding.lower()
        data = np.frombuffer(msg.data, dtype=np.uint8)

        try:
            if enc in ('bgr8', '8uc3'):
                return data.reshape((msg.height, msg.width, 3))

            if enc == 'rgb8':
                rgb = data.reshape((msg.height, msg.width, 3))
                return rgb[:, :, ::-1]

            if enc == 'bgra8':
                bgra = data.reshape((msg.height, msg.width, 4))
                return bgra[:, :, :3]

            if enc == 'rgba8':
                rgba = data.reshape((msg.height, msg.width, 4))
                return rgba[:, :, [2, 1, 0]]

        except ValueError:
            self.get_logger().warn(
                f"Bad image buffer size for encoding={msg.encoding}, w={msg.width}, h={msg.height}"
            )
            return None

        self.get_logger().warn(f"Unsupported image encoding: {msg.encoding}")
        return None

    """ Canny Edge Detection """

    def blur_image(self, image, kernel):
        image_array = np.array(image, dtype=np.float32)
        return self.convolve2d(image_array, kernel)

    def convolve2d(self, image, kernel):
        image = np.array(image, dtype=np.float64)
        kernel = np.array(kernel, dtype=np.float64)

        k_h, k_w = kernel.shape
        pad_h = k_h // 2
        pad_w = k_w // 2

        padded_image = np.pad(image, ((pad_h, pad_h), (pad_w, pad_w)), mode='edge')

        i_h, i_w = image.shape
        stride_window = (i_h, i_w, k_h, k_w)

        s_h, s_w = padded_image.strides
        view_strides = (s_h, s_w, s_h, s_w)

        windows = as_strided(padded_image, shape=stride_window, strides=view_strides)

        return np.sum(windows * kernel, axis=(2, 3))

    # Sobel filter
    def sobel_filters(self, image):
        image_array = np.array(image, dtype=np.float32)

        Gx = np.array([
            [-1, 0, 1],
            [-2, 0, 2],
            [-1, 0, 1],
        ], dtype=np.float32)

        Gy = np.array([
            [1, 2, 1],
            [0, 0, 0],
            [-1, -2, -1],
        ], dtype=np.float32)

        gradient_x = self.convolve2d(image_array, Gx)
        gradient_y = self.convolve2d(image_array, Gy)

        gradient_magnitude = np.hypot(gradient_x, gradient_y)
        gradient_direction = np.arctan2(gradient_y, gradient_x)

        return gradient_magnitude, gradient_direction

    def non_max_suppression(self, magnitude, direction):
        M, N = magnitude.shape
        Z = np.zeros((M, N), dtype=np.float32)

        # Normalize angles
        angle = np.rad2deg(direction) % 180

        mag = magnitude

        # Shifted images
        magN  = np.pad(mag, 1, mode='edge')

        # neighbors 
        d0   = (angle < 22.5) | (angle >= 157.5)
        d45  = (22.5 <= angle) & (angle < 67.5)
        d90  = (67.5 <= angle) & (angle < 112.5)
        d135 = (112.5 <= angle) & (angle < 157.5)

        # For each direction define neighbors
        West  = magN[1:M+1, 0:N]
        East = magN[1:M+1, 2:N+2]

        North    = magN[0:M,   1:N+1]
        South  = magN[2:M+2, 1:N+1]

        NW    = magN[0:M,   0:N]
        NE   = magN[0:M,   2:N+2]
        SW  = magN[2:M+2, 0:N]
        SE = magN[2:M+2, 2:N+2]

        # Performing non_max_supression
        store0 = d0  & (mag >= West) & (mag >= East)
        store90 = d90 & (mag >= North)   & (mag >= South)

        store45 = d45 & (mag >= NE) & (mag >= SW)
        store135 = d135 & (mag >= NW) & (mag >= SE)

        # Combine
        store = store0 | store90 | store45 | store135
        Z[store] = mag[store]

        return Z

    # Double thresholding
    def double_thresholding(self, image):
        highThreshold = 166 
        lowThreshold = 89 

        M, N = image.shape
        res = np.zeros((M,N), dtype=np.int32)

        strong = np.int32(255)
        weak = np.int32(75)
        
        strong_x, strong_y = np.where(image >= highThreshold)
        weak_x, weak_y = np.where((image <= highThreshold) & (image >= lowThreshold))

        res[strong_x, strong_y] = strong
        res[weak_x, weak_y] = weak

        return res, weak, strong

    # Edge tracking by hysteresis
    def hysteresis(self, image, weak, strong=255):
        img = image.copy()

        # Start with only strong pixels
        strong_mask = (img == strong)
        weak_mask = (img == weak)

        changed = True
        while changed:
            # Grow strong region by 1 pixel in all directions
            grown = np.pad(strong_mask, 1, mode="edge")
            grown = (
                grown[:-2, :-2] | grown[:-2, 1:-1] | grown[:-2, 2:] |
                grown[1:-1, :-2] | grown[1:-1, 1:-1] | grown[1:-1, 2:] |
                grown[2:, :-2] | grown[2:, 1:-1] | grown[2:, 2:]
            )

            # weak pixels adjacent to strong become strong
            new_strong = grown & weak_mask

            if not new_strong.any():
                changed = False
            else:
                strong_mask |= new_strong
                weak_mask &= ~new_strong

        # final image
        out = np.zeros_like(img)
        out[strong_mask] = strong
        return out

    # Complete Canny edge detection; accepts numpy BGR/RGB or grayscale frame.
    def canny_edge_detection(self, raw_image):
        raw_np = np.array(raw_image)

        # Step 1: grayscale
        if raw_np.ndim == 2:
            gray = raw_np.astype(np.float32)
        else:
            # Inputs in this node are BGR; this weighted mix is robust enough for edges.
            b = raw_np[:, :, 0].astype(np.float32)
            g = raw_np[:, :, 1].astype(np.float32)
            r = raw_np[:, :, 2].astype(np.float32)
            gray = 0.114 * b + 0.587 * g + 0.299 * r

        # Step 2: blur then gradient
        K = (1.0 / 9.0) * np.ones((3, 3), dtype=np.float32)
        blurred = self.convolve2d(gray, K)
        gradient_magnitude, gradient_direction = self.sobel_filters(blurred)

        # Step 3: non-maximum suppression
        non_max_img = self.non_max_suppression(gradient_magnitude, gradient_direction)

        # Step 4: double threshold
        thresholded_img, weak, strong = self.double_thresholding(non_max_img)

        # Step 5: hysteresis
        final_img = self.hysteresis(thresholded_img, weak, strong)

        # Return uint8 edge map (0 or 255)
        return np.uint8(final_img)

    


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LineFollowerV2()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()