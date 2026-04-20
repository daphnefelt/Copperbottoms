import rclpy
import numpy as np
import os
from numpy.lib.stride_tricks import as_strided
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

try:
    import cv2
    CV2_AVAILABLE = True
except Exception:
    cv2 = None
    CV2_AVAILABLE = False


class LineFollowerV2(Node):

    def __init__(self):
        super().__init__('line_follower_v2')

        # --- Topics ---
        self.image_sub = self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)
        self.armed_sub = self.create_subscription(Bool, '/rover/armed', self.armed_callback, 10)
        self.vel_pub   = self.create_publisher(Twist, '/cmd_vel', 10)
        self.debug_pub = self.create_publisher(Image, '/line_follower/debug_image', 10)

        # --- Drive params ---
        self.forward_speed  = 0.25
        self.search_speed   = 0.0
        self.search_turn    = 0.35
        self.max_turn       = 1.0
        self.kp             = 0.80
        self.error_offset   = float(os.environ.get('LF_ERROR_OFFSET', '-0.40'))
        self.err_alpha      = 0.18   # low-pass on error signal
        self.x_alpha        = 0.22   # low-pass on track centroid X
        self.max_turn_step  = 0.05   # angular rate limiter

        # --- Corner / turn params ---
        # corner_shift_thresh: normalized centroid-X shift that signals a corner.
        # Compared between a NARROW top band (top 30%) and the bottom section (bottom 50%).
        # Using a narrow top band means even a short horizontal stretch entering the frame
        # dominates the top centroid while the long vertical approach dominates the bottom.
        # A dead zone between the two bands avoids the "knee" of the L confusing both.
        self.corner_top_end          = 0.30  # top band: rows 0  → 30%
        self.corner_bot_start        = 0.50  # bot band: rows 50% → 100%
        self.corner_shift_thresh     = 0.22  # raised from 0.15 — 15% was too sensitive at startup angles
        self.corner_confirm_frames   = 3    # consecutive detections before committing
        self.corner_min_follow_frames = 10  # must be in follow for N frames before corner can trigger
        self.corner_commit_frames    = 15   # frames to blind-turn before reacquisition check is allowed
        self.corner_reacquire_frames = 4    # consecutive frames passing exit gates before exiting
        self.corner_reacquire_px     = 70   # min track pixels to count as reacquired
        self.corner_clear_frames     = 5    # corner detector must be clear for N frames before exit counts
        self.corner_max_frames       = 60   # safety timeout — raised so long turns don't force-exit
        self.corner_turn_speed       = 0.25   # forward speed during blind turn phase
        self.corner_turn_rate        = 0.65   # angular rate during blind commit phase (raised — guided phase corrects overshoot now)
        self.corner_guided_speed     = 0.20   # forward speed during guided phase (slightly slower for accuracy)
        self.corner_guided_kp        = 1.20   # steering gain toward tape centroid during guided phase
        self.corner_guided_bias      = 0.25   # minimum angular push in turn direction during guided phase
                                              # prevents rover stalling if tape briefly disappears mid-guide

        # --- Track / ROI params ---
        self.roi_top_ratio    = 0.55   # follow control only uses lower portion of frame
        self.min_track_px     = 70
        self.track_lock_req   = 2      # frames of track presence before entering follow
        self.acquire_speed    = 0.08

        # --- Bridge (gap crossing) params ---
        # When the ROI loses the line, scan the full image before spinning.
        # If blue is visible anywhere, creep forward steering toward it.
        # Only fall through to spin-search if the full image also has nothing.
        self.bridge_speed         = 0.15   # forward speed while bridging a gap
        self.bridge_kp            = 0.60   # proportional steering toward full-image centroid
        self.bridge_min_px        = 120    # raised from 40 — noise was causing constant bridge cycling
        self.bridge_max_blank     = 15     # frames with no full-image blue before -> search

        # --- Blue color params (calibrated for blue tape, BGR space) ---
        self.target_bgr         = np.array([164.0, 108.0, 7.0], dtype=np.float32)
        self.color_tol          = np.array([50.0,  50.0,  90.0], dtype=np.float32)
        self.blue_score_thresh  = 15.0
        self.simple_blue_thresh = 75.0   # raised from 58 — 17k px on ROI means noise is being picked up
        self.edge_boost         = 1.6    # weight multiplier on edge-aligned pixels
        self.min_col_peak       = 2.5

        # --- Component selector params ---
        self.comp_min_area     = 70
        self.comp_bottom_rows  = 10
        self.comp_bottom_bonus = 1.8

        # --- Canny params ---
        self.canny_high_pct  = 86.0
        self.canny_low_ratio = 0.45

        # --- Debug ---
        self.debug_topic   = os.environ.get('LF_DEBUG_TOPIC', '0') == '1'
        self.debug_every_n = max(int(os.environ.get('LF_DEBUG_EVERY', '1')), 1)

        # --- Runtime state ---
        self.armed        = False
        self.frame_count  = 0
        self.state        = 'search'
        self.last_error   = 0.0
        self.last_turn    = 0.0
        self.last_speed   = 0.0
        self.last_unbiased = 0.0
        self.last_biased   = 0.0
        self.track_x_filt = None
        self.lost_frames  = 0
        self.lock_frames  = 0
        # Corner sub-state
        self.corner_streak       = 0
        self.corner_clear_streak = 0   # consecutive frames corner_det has been False during turn
        self.corner_sign         = 1.0
        self.turn_commit_left    = 0
        self.turn_reacq          = 0
        self.turn_frames         = 0
        self.bridge_blank_frames   = 0   # consecutive frames with no full-image blue
        self.follow_frames_total    = 0   # total frames spent in follow state (guards corner trigger)

        self.get_logger().info('LineFollower started.')

    # =========================================================================
    # ROS Callbacks
    # =========================================================================

    def armed_callback(self, msg: Bool):
        prev = self.armed
        self.armed = bool(msg.data)
        if self.armed != prev:
            self.get_logger().info(f'Armed: {self.armed}')

    def image_callback(self, msg: Image):
        self.frame_count += 1
        img = self.decode_image(msg)
        if img is None:
            return

        if not self.armed:
            self.publish_stop()
            return

        h, w, _ = img.shape

        # ROI for follow control (lower portion — stable, less perspective distortion)
        roi_y  = int(h * self.roi_top_ratio)
        roi    = img[roi_y:, :, :]

        blue_mask, blue_score = self.blue_mask_and_score(roi)
        blue_mask  = self.clean_mask(blue_mask)
        edge_map   = self.canny(roi)
        track_mask = self.best_component(blue_mask, blue_score)
        track_px   = int(np.sum(track_mask))

        # Full-image blue mask for corner detection (need top of frame for L-shape)
        blue_full_raw, blue_score_full = self.blue_mask_and_score(img)
        blue_full_raw = self.clean_mask(blue_full_raw)
        blue_full     = self.best_component(blue_full_raw, blue_score_full)

        # ---- Corner detection ----
        corner_det, candidate_sign, shift_val = self.detect_corner(blue_full)
        if corner_det:
            self.corner_streak += 1
            if self.state != 'turn':
                self.corner_sign = candidate_sign   # lock direction on entry only
        else:
            self.corner_streak = 0

        corner_confirmed = self.corner_streak >= self.corner_confirm_frames

        # Enter turn state when corner is confirmed, we have enough track, AND
        # we've been following long enough that startup geometry can't false-trigger.
        corner_armed = self.follow_frames_total >= self.corner_min_follow_frames
        if self.state != 'turn' and corner_confirmed and track_px >= self.min_track_px and corner_armed:
            self.state          = 'turn'
            self.turn_commit_left    = self.corner_commit_frames
            self.turn_reacq          = 0
            self.corner_clear_streak = 0
            self.turn_frames         = 0
            dir_str = 'LEFT' if self.corner_sign > 0 else 'RIGHT'
            self.get_logger().warn(
                f'[TURN ENTER] dir={dir_str}  sign={self.corner_sign:+.0f}  '
                f'shift={shift_val:.3f}  streak={self.corner_streak}  track_px={track_px}'
            )

        # ---- Turn state execution ----
        if self.state == 'turn':
            self.turn_frames += 1
            if self.turn_commit_left > 0:
                self.turn_commit_left -= 1

            # Track how long corner_det has been False — need it gone for N frames
            # before exit can count. This prevents exiting mid-turn when the detector
            # briefly drops out before the rover has actually cleared the corner.
            if corner_det:
                self.corner_clear_streak = 0
            else:
                self.corner_clear_streak += 1

            # Exit requires ALL three:
            #   1. commit period over (blind turn done)
            #   2. line visible in ROI again
            #   3. corner shape gone for N consecutive frames
            # NOTE: vertical aspect ratio gate was removed — diagonal post-turn tape
            # has similar bbox width/height and was blocking exit every run.
            if (self.turn_commit_left <= 0
                    and track_px >= self.corner_reacquire_px
                    and self.corner_clear_streak >= self.corner_clear_frames):
                self.turn_reacq += 1
            else:
                self.turn_reacq = 0

            if self.frame_count % 5 == 0:
                self.get_logger().info(
                    f'[TURN] frame={self.turn_frames}/{self.corner_max_frames}  '
                    f'commit_left={self.turn_commit_left}  '
                    f'reacq={self.turn_reacq}/{self.corner_reacquire_frames}  '
                    f'track_px={track_px}  corner_det={corner_det}  '
                    f'clear={self.corner_clear_streak}/{self.corner_clear_frames}  '
                    f'shift={shift_val:.3f}'
                )

            if self.turn_reacq >= self.corner_reacquire_frames:
                self._exit_turn('reacquired', track_px)
            elif self.turn_frames >= self.corner_max_frames:
                self._exit_turn('safety timeout', track_px)
            else:
                self.do_turn(blue_full, w)
                self.publish_debug(roi, blue_mask, edge_map, track_mask, track_px,
                                   shift_val, corner_det, msg.header)
                return

        # ---- Follow / Bridge / Search ----
        if track_px >= self.min_track_px:
            # Line visible in ROI — normal follow path
            self.lock_frames += 1
            self.bridge_blank_frames = 0
            if self.lock_frames < self.track_lock_req:
                self.state = 'acquire'
                self.publish_acquire()
                if self.frame_count % 10 == 0:
                    self.get_logger().info(
                        f'[ACQUIRE] lock={self.lock_frames}/{self.track_lock_req}  '
                        f'track_px={track_px}'
                    )
            else:
                track_x    = self.track_center_x(track_mask, blue_score, edge_map > 0)
                self.state = 'follow'
                self.follow_frames_total += 1
                self.do_follow(track_x, roi.shape[1])
                if self.frame_count % 10 == 0:
                    corner_ready = self.follow_frames_total >= self.corner_min_follow_frames
                    self.get_logger().info(
                        f'[FOLLOW] lin={self.last_speed:.2f}  ang={self.last_turn:.2f}  '
                        f'err_unbiased={self.last_unbiased:.3f}  err_biased={self.last_biased:.3f}  '
                        f'offset={self.error_offset:.2f}  track_px={track_px}  '
                        f'corner_shift={shift_val:.3f}  streak={self.corner_streak}  '
                        f'corner_armed={corner_ready}({self.follow_frames_total}/{self.corner_min_follow_frames})'
                    )
        else:
            # ROI line lost — check the full image before giving up and spinning
            self.lock_frames         = 0
            self.track_x_filt        = None
            self.follow_frames_total = 0   # reset so corner can't fire on re-acquisition before stable

            full_blue_px = int(np.sum(blue_full))
            if full_blue_px >= self.bridge_min_px:
                # Blue is visible somewhere in the full frame — creep toward it
                self.bridge_blank_frames = 0
                self.state = 'bridge'
                self.do_bridge(blue_full, w)
                if self.frame_count % 10 == 0:
                    self.get_logger().info(
                        f'[BRIDGE] full_blue_px={full_blue_px}  '
                        f'ang={self.last_turn:.2f}  lin={self.last_speed:.2f}'
                    )
            else:
                # Nothing visible anywhere — spin and hunt
                self.bridge_blank_frames += 1
                self.state = 'search'
                self.do_search()
                if self.bridge_blank_frames == 1:
                    self.get_logger().warn('[BRIDGE→SEARCH] no blue in full image, spinning')

        self.publish_debug(roi, blue_mask, edge_map, track_mask, track_px,
                           shift_val, corner_det, msg.header)

    def _exit_turn(self, reason: str, track_px: int = 0):
        dir_str = 'LEFT' if self.corner_sign > 0 else 'RIGHT'
        self.get_logger().warn(
            f'[TURN EXIT] reason={reason}  dir={dir_str}  '
            f'frames={self.turn_frames}  track_px={track_px}'
        )
        self.state               = 'follow'
        self.corner_streak       = 0
        self.corner_clear_streak = 0
        self.turn_reacq          = 0
        self.turn_frames         = 0

    # =========================================================================
    # Drive Commands
    # =========================================================================

    def publish_stop(self):
        self.vel_pub.publish(Twist())

    def publish_acquire(self):
        cmd = Twist()
        cmd.linear.x = self.acquire_speed
        self.vel_pub.publish(cmd)

    def do_follow(self, track_x: int, width: int):
        if self.track_x_filt is None:
            self.track_x_filt = float(track_x)
        else:
            self.track_x_filt = (1 - self.x_alpha) * self.track_x_filt + self.x_alpha * track_x

        cx       = width / 2.0
        unbiased = (self.track_x_filt - cx) / max(cx, 1.0)
        raw      = unbiased + self.error_offset
        if abs(raw) < 0.03:
            raw = 0.0

        biased = (1 - self.err_alpha) * self.last_error + self.err_alpha * raw
        self.last_error    = biased
        self.last_unbiased = unbiased
        self.last_biased   = biased

        turn = float(np.clip(-self.kp * biased, -self.max_turn, self.max_turn))
        turn = float(np.clip(turn, self.last_turn - self.max_turn_step,
                                   self.last_turn + self.max_turn_step))

        cmd = Twist()
        cmd.linear.x  = self.forward_speed
        cmd.angular.z = turn
        self.vel_pub.publish(cmd)
        self.last_speed  = cmd.linear.x
        self.last_turn   = cmd.angular.z
        self.lost_frames = 0

    def do_search(self):
        self.lost_frames += 1
        spin = -1.0 if self.last_turn >= 0 else 1.0   # keep spinning toward last known line
        cmd = Twist()
        cmd.linear.x  = self.search_speed
        cmd.angular.z = spin * self.search_turn
        self.vel_pub.publish(cmd)
        if self.lost_frames % 10 == 0:
            spin_dir = 'LEFT' if spin > 0 else 'RIGHT'
            self.get_logger().warn(
                f'[SEARCH] lost_frames={self.lost_frames}  '
                f'spinning={spin_dir}  ang={cmd.angular.z:.2f}'
            )

    def do_bridge(self, blue_full: np.ndarray, img_width: int):
        """Creep forward steering toward the centroid of blue pixels in the full image.
        Used to cross intentional gaps in the tape without losing the line entirely."""
        full_px = int(np.sum(blue_full))
        if full_px > 0:
            cols   = np.arange(img_width, dtype=np.float32)
            cx_full = float(np.dot(cols, blue_full.sum(axis=0))) / full_px
            # Normalize error to [-1, 1] same as follow
            error  = (cx_full - img_width / 2.0) / max(img_width / 2.0, 1.0)
            turn   = float(np.clip(-self.bridge_kp * error, -self.max_turn, self.max_turn))
        else:
            turn = self.last_turn  # hold last heading if centroid vanishes mid-call

        cmd = Twist()
        cmd.linear.x  = self.bridge_speed
        cmd.angular.z = turn
        self.vel_pub.publish(cmd)
        self.last_speed = cmd.linear.x
        self.last_turn  = cmd.angular.z

    def do_turn(self, blue_full: np.ndarray, img_width: int):
        """
        Two-phase turn:
          Phase 1 — blind commit (turn_commit_left > 0):
            Fixed rate spin. Gets rover around the bulk of the corner regardless
            of where the tape is. Duration set by corner_commit_frames.

          Phase 2 — guided turn (turn_commit_left == 0):
            Steer toward full-image blue centroid like bridge mode, but with a
            minimum bias in the original turn direction so the rover keeps moving
            around the corner even if the tape briefly disappears.
            Self-corrects both overshoot and undershoot automatically.
        """
        cmd = Twist()

        if self.turn_commit_left > 0:
            # Phase 1: blind fixed-rate spin
            cmd.linear.x  = self.corner_turn_speed
            cmd.angular.z = float(np.clip(
                self.corner_sign * self.corner_turn_rate, -self.max_turn, self.max_turn
            ))
            phase = 'blind'
        else:
            # Phase 2: guide toward tape centroid
            full_px = int(np.sum(blue_full))
            if full_px > 0:
                cols    = np.arange(img_width, dtype=np.float32)
                cx_full = float(np.dot(cols, blue_full.sum(axis=0))) / full_px
                error   = (cx_full - img_width / 2.0) / max(img_width / 2.0, 1.0)
                # P steer toward centroid, then add a directional bias so we
                # keep turning even when tape is briefly near center during transition
                guided_ang = -self.corner_guided_kp * error
                bias       = self.corner_sign * self.corner_guided_bias
                ang        = float(np.clip(guided_ang + bias, -self.max_turn, self.max_turn))
            else:
                # No tape visible — hold turn direction with reduced rate
                ang = float(np.clip(
                    self.corner_sign * self.corner_guided_bias, -self.max_turn, self.max_turn
                ))
            cmd.linear.x  = self.corner_guided_speed
            cmd.angular.z = ang
            phase = 'guided'

        self.vel_pub.publish(cmd)
        self.last_speed = cmd.linear.x
        self.last_turn  = cmd.angular.z

        if self.turn_frames % 5 == 1:
            self.get_logger().warn(
                f'[DO_TURN:{phase}] lin={cmd.linear.x:.2f}  ang={cmd.angular.z:.2f}  '
                f'sign={self.corner_sign:.0f}  commit_left={self.turn_commit_left}'
            )

    # =========================================================================
    # Corner Detection  (centroid-shift method)
    # =========================================================================

    def detect_corner(self, mask: np.ndarray):
        """
        Detect corner by comparing horizontal centroid between a narrow top band
        and the bottom section of the full-image blue mask.

        Why not top-half vs bottom-half: when approaching a corner, the horizontal
        piece may only occupy a short stretch at the very top of the frame. If we
        use the top HALF, vertical pixels in that half dilute the centroid and the
        shift falls below threshold. A narrow top band (top 30%) lets even a short
        horizontal piece dominate the centroid calculation. The dead zone between
        the two bands (30-50%) excludes the "knee" of the L, which otherwise pulls
        both centroids toward the bend and reduces the apparent shift.

        Two cases this handles:
          Approaching: short horizontal at top of frame → clearly dominates narrow band
          Sitting on corner: horizontal fills top band entirely → large shift

        Returns (detected: bool, turn_sign: float)
          turn_sign: +1.0 = left (CCW), -1.0 = right (CW)
        """
        h, w      = mask.shape
        top_end   = int(h * self.corner_top_end)    # e.g. row 0  → 30%
        bot_start = int(h * self.corner_bot_start)  # e.g. row 50% → end

        top = mask[:top_end,   :]
        bot = mask[bot_start:, :]

        top_px = int(np.sum(top))
        bot_px = int(np.sum(bot))

        # Lower pixel floor for top since it covers fewer rows
        if top_px < 15 or bot_px < 20:
            return False, 0.0, 0.0

        cols   = np.arange(w, dtype=np.float32)
        top_cx = float(np.dot(cols, top.sum(axis=0))) / top_px
        bot_cx = float(np.dot(cols, bot.sum(axis=0))) / bot_px

        shift = (top_cx - bot_cx) / w   # normalized, resolution-independent

        if abs(shift) > self.corner_shift_thresh:
            # Positive shift → top centroid is RIGHT of bottom → tape bends right → turn right (sign=-1)
            # Negative shift → top centroid is LEFT  of bottom → tape bends left  → turn left  (sign=+1)
            turn_sign = 1.0 if shift > 0 else -1.0
            return True, turn_sign, shift

        return False, 0.0, shift

    # =========================================================================
    # Image Processing
    # =========================================================================

    def blue_mask_and_score(self, roi: np.ndarray):
        f = roi.astype(np.float32)
        b, g, r = f[:, :, 0], f[:, :, 1], f[:, :, 2]

        blue_score    = b - 0.5 * (g + r)
        channel_order = (b > g + 8.0) & (b > r + 25.0)

        near_target = np.all(np.abs(f - self.target_bgr) <= self.color_tol, axis=2)
        tuned       = near_target & (blue_score > self.blue_score_thresh) & channel_order
        simple      = (blue_score > self.simple_blue_thresh) & channel_order

        return tuned | simple, blue_score

    def clean_mask(self, mask: np.ndarray) -> np.ndarray:
        """2-pass majority filter to suppress noise."""
        m = mask.astype(np.uint8)
        for _ in range(2):
            p = np.pad(m, 1, mode='edge')
            neighbors = (p[:-2, :-2] + p[:-2, 1:-1] + p[:-2, 2:] +
                         p[1:-1, :-2] + p[1:-1, 1:-1] + p[1:-1, 2:] +
                         p[2:,  :-2] + p[2:,  1:-1] + p[2:,  2:])
            m = (neighbors >= 4).astype(np.uint8)
        return m.astype(bool)

    def best_component(self, mask: np.ndarray, blue_score: np.ndarray) -> np.ndarray:
        """Keep only the most track-like connected component."""
        if not CV2_AVAILABLE or mask.size == 0:
            return mask
        n, labels, stats, centroids = cv2.connectedComponentsWithStats(
            mask.astype(np.uint8) * 255, connectivity=8)
        if n <= 1:
            return mask

        h, w       = mask.shape
        best_lbl   = 0
        best_score = -1.0

        for lbl in range(1, n):
            area = int(stats[lbl, cv2.CC_STAT_AREA])
            if area < self.comp_min_area:
                continue
            comp = labels == lbl
            cx   = float(centroids[lbl][0])
            ref  = self.track_x_filt if self.track_x_filt is not None else cx
            continuity  = max(0.2, 1.0 - abs(cx - ref) / max(w * 0.6, 1.0))
            has_bottom  = bool(np.any(comp[h - self.comp_bottom_rows:, :]))
            score       = area * continuity * (self.comp_bottom_bonus if has_bottom else 1.0)
            if score > best_score:
                best_score = score
                best_lbl   = lbl

        return (labels == best_lbl) if best_lbl else mask

    def track_center_x(self, mask: np.ndarray, blue_score: np.ndarray, edge_mask: np.ndarray) -> int:
        weighted = np.maximum(blue_score, 0).astype(np.float32)
        weighted *= (1.0 + self.edge_boost * edge_mask.astype(np.float32))
        weighted *= mask.astype(np.float32)
        col_str = weighted.mean(axis=0)
        if col_str.size > 0 and float(col_str.max()) >= self.min_col_peak:
            return int(np.argmax(col_str))
        ys, xs = np.where(mask)
        return int(np.mean(xs)) if xs.size > 0 else mask.shape[1] // 2

    def decode_image(self, msg: Image):
        enc  = msg.encoding.lower()
        data = np.frombuffer(msg.data, dtype=np.uint8)
        try:
            if enc in ('bgr8', '8uc3'):
                return data.reshape((msg.height, msg.width, 3))
            if enc == 'rgb8':
                return data.reshape((msg.height, msg.width, 3))[:, :, ::-1]
            if enc == 'bgra8':
                return data.reshape((msg.height, msg.width, 4))[:, :, :3]
            if enc == 'rgba8':
                return data.reshape((msg.height, msg.width, 4))[:, :, [2, 1, 0]]
        except ValueError:
            self.get_logger().warn(f'Bad image buffer: enc={enc} shape=({msg.height},{msg.width})')
            return None
        self.get_logger().warn(f'Unsupported encoding: {msg.encoding}')
        return None

    # =========================================================================
    # Debug Image
    # =========================================================================

    def publish_debug(self, roi, blue_mask, edge_map, track_mask,
                      track_px, shift_val, corner_det, header):
        if not self.debug_topic or not CV2_AVAILABLE:
            return
        if self.frame_count % self.debug_every_n != 0:
            return
        try:
            h, w = roi.shape[:2]
            vis  = roi.copy()
            cv2.line(vis, (w // 2, 0), (w // 2, h - 1), (0, 255, 0), 2)
            if self.track_x_filt is not None:
                tx = int(np.clip(self.track_x_filt, 0, w - 1))
                cv2.line(vis, (tx, 0), (tx, h - 1), (0, 0, 255), 2)

            def g2bgr(m):
                return cv2.cvtColor(m.astype(np.uint8), cv2.COLOR_GRAY2BGR)

            blue_vis  = g2bgr(blue_mask.astype(np.uint8) * 255)
            edge_vis  = g2bgr(edge_map)
            track_vis = g2bgr(track_mask.astype(np.uint8) * 255)

            # Label each panel
            for panel, name in ((vis,       'ROI'),
                                (blue_vis,  'Blue Mask'),
                                (edge_vis,  'Edges'),
                                (track_vis, 'Track')):
                cv2.putText(panel, name, (6, 18),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 1)

            grid = np.vstack((
                np.hstack((vis,      blue_vis)),
                np.hstack((edge_vis, track_vis)),
            ))

            # Two-line telemetry bar at bottom
            bar_h  = 44
            canvas = np.zeros((grid.shape[0] + bar_h, grid.shape[1], 3), dtype=np.uint8)
            canvas[:grid.shape[0]] = grid

            state_color = {
                'follow':  (100, 255, 100),
                'turn':    (100, 100, 255),
                'search':  (100, 200, 255),
                'acquire': (255, 200, 100),
                'bridge':  (255, 255,   0),   # yellow — crossing a gap
            }.get(self.state, (200, 200, 200))

            line1 = (f'STATE={self.state.upper()}  '
                     f'ang={self.last_turn:.2f}  lin={self.last_speed:.2f}  '
                     f'err_b={self.last_biased:.3f}  err_u={self.last_unbiased:.3f}  '
                     f'offset={self.error_offset:.2f}')
            line2 = (f'track_px={track_px}  '
                     f'corner_shift={shift_val:.3f}(thresh={self.corner_shift_thresh})  '
                     f'corner_det={corner_det}  streak={self.corner_streak}  '
                     f'turn_frame={self.turn_frames}  reacq={self.turn_reacq}')

            y0 = grid.shape[0]
            cv2.putText(canvas, line1, (8, y0 + 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.50, state_color, 1)
            cv2.putText(canvas, line2, (8, y0 + 34),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.50, (200, 220, 255), 1)

            out          = Image()
            out.header   = header
            out.height   = int(canvas.shape[0])
            out.width    = int(canvas.shape[1])
            out.encoding = 'bgr8'
            out.step     = int(canvas.shape[1] * 3)
            out.data     = canvas.tobytes()
            self.debug_pub.publish(out)
        except Exception as e:
            self.get_logger().warn(f'Debug publish failed: {e}')

    # =========================================================================
    # Canny Edge Detection (pure NumPy)
    # =========================================================================

    def canny(self, img: np.ndarray) -> np.ndarray:
        arr = img.astype(np.float32)
        if arr.ndim == 3:
            gray = 0.114 * arr[:, :, 0] + 0.587 * arr[:, :, 1] + 0.299 * arr[:, :, 2]
        else:
            gray = arr
        blurred = self._conv2d(gray, np.ones((3, 3), dtype=np.float32) / 9.0)
        mag, direction = self._sobel(blurred)
        nms             = self._nms(mag, direction)
        thresh, wk, st  = self._double_threshold(nms)
        return self._hysteresis(thresh, wk, st).astype(np.uint8)

    def _conv2d(self, img: np.ndarray, kernel: np.ndarray) -> np.ndarray:
        img    = img.astype(np.float64)
        kernel = kernel.astype(np.float64)
        kh, kw = kernel.shape
        padded = np.pad(img, (kh // 2, kw // 2), mode='edge')
        ih, iw = img.shape
        windows = as_strided(padded, shape=(ih, iw, kh, kw),
                             strides=padded.strides * 2)
        return np.einsum('ijkl,kl->ij', windows, kernel)

    def _sobel(self, img: np.ndarray):
        Gx = np.array([[-1, 0, 1], [-2, 0, 2], [-1, 0, 1]], dtype=np.float32)
        Gy = np.array([[ 1, 2, 1], [ 0, 0, 0], [-1,-2,-1]], dtype=np.float32)
        gx = self._conv2d(img, Gx)
        gy = self._conv2d(img, Gy)
        return np.hypot(gx, gy), np.arctan2(gy, gx)

    def _nms(self, mag: np.ndarray, direction: np.ndarray) -> np.ndarray:
        M, N  = mag.shape
        angle = np.rad2deg(direction) % 180
        p     = np.pad(mag, 1, mode='edge')
        W,  E  = p[1:M+1, 0:N],   p[1:M+1, 2:N+2]
        Nn, S  = p[0:M,   1:N+1], p[2:M+2, 1:N+1]
        NE, SW = p[0:M,   2:N+2], p[2:M+2, 0:N]
        NW, SE = p[0:M,   0:N],   p[2:M+2, 2:N+2]

        keep = (((angle <  22.5) | (angle >= 157.5)) & (mag >= W)  & (mag >= E)  |
                ((angle >= 67.5) & (angle < 112.5))  & (mag >= Nn) & (mag >= S)  |
                ((angle >= 22.5) & (angle <  67.5))  & (mag >= NE) & (mag >= SW) |
                ((angle >=112.5) & (angle < 157.5))  & (mag >= NW) & (mag >= SE))
        Z = np.zeros_like(mag)
        Z[keep] = mag[keep]
        return Z

    def _double_threshold(self, img: np.ndarray):
        strong, weak = np.int32(255), np.int32(75)
        res = np.zeros_like(img, dtype=np.int32)
        nz  = img[img > 0]
        if nz.size == 0:
            return res, weak, strong
        hi = max(float(np.percentile(nz, self.canny_high_pct)), 30.0)
        lo = max(8.0, hi * self.canny_low_ratio)
        res[img >= hi] = strong
        res[(img < hi) & (img >= lo)] = weak
        return res, weak, strong

    def _hysteresis(self, img: np.ndarray, weak: int, strong: int = 255) -> np.ndarray:
        strong_m = img == strong
        weak_m   = img == weak
        changed  = True
        while changed:
            p = np.pad(strong_m, 1, mode='edge')
            grown = (p[:-2, :-2] | p[:-2, 1:-1] | p[:-2, 2:] |
                     p[1:-1,:-2] | p[1:-1,1:-1] | p[1:-1,2:] |
                     p[2:,  :-2] | p[2:,  1:-1] | p[2:,  2:])
            new_strong = grown & weak_m
            if not new_strong.any():
                changed = False
            else:
                strong_m |= new_strong
                weak_m   &= ~new_strong
        out = np.zeros_like(img)
        out[strong_m] = strong
        return out


# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerV2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if CV2_AVAILABLE:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
