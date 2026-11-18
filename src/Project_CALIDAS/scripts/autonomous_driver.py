#!/usr/bin/env python3
"""
Sub-Project 1 (CA): Autonomous Monocular Vision Driver for Project_CALIDAS
Calibrated for camera_joint origin: xyz="0.7 0 0.17" rpy="0 0 0"
Target Environment: Ubuntu 24.04 | ROS 2 Jazzy | Gazebo Harmonic
Input:  /camera/image_raw (sensor_msgs/msg/Image, 640x480 RGB8 @ 10 Hz)
Output: /cmd_vel (geometry_msgs/msg/Twist)
Debug:  /camera/debug_image (sensor_msgs/msg/Image)
"""

import math
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


class AutonomousTrackDriver(Node):
    def __init__(self):
        super().__init__('autonomous_track_driver')

        # ---------------- Tunable Parameters ----------------
        self.declare_parameter('wheel_base', 0.76)          # meters (from gazebo_control_car.xacro)
        self.declare_parameter('max_steer_angle', 0.58)     # radians (matches 0.6 rad URDF limit)
        self.declare_parameter('max_speed', 1.65)           # m/s on straightaways
        self.declare_parameter('min_speed', 0.70)           # m/s in sharp hairpins
        self.declare_parameter('search_speed', 0.55)        # m/s during apex recovery
        self.declare_parameter('kp_near', 0.52)             # proportional gain on near lookahead error
        self.declare_parameter('kp_far', 0.48)              # proportional gain on far lookahead error
        self.declare_parameter('kd_steer', 0.14)            # derivative damping gain
        self.declare_parameter('steering_smoothing', 0.65)  # EMA low-pass filter weight (0..1)
        self.declare_parameter('show_cv_window', True)      # pop up live OpenCV HUD window

        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.max_steer = float(self.get_parameter('max_steer_angle').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.min_speed = float(self.get_parameter('min_speed').value)
        self.search_speed = float(self.get_parameter('search_speed').value)
        self.kp_near = float(self.get_parameter('kp_near').value)
        self.kp_far = float(self.get_parameter('kp_far').value)
        self.kd_steer = float(self.get_parameter('kd_steer').value)
        self.alpha_steer = float(self.get_parameter('steering_smoothing').value)
        self.show_cv_window = bool(self.get_parameter('show_cv_window').value)

        # ---------------- Controller State ----------------
        self.prev_error = 0.0
        self.prev_time = self.get_clock().now()
        self.smoothed_steer = 0.0
        self.last_valid_steer = 0.0
        self.last_road_cx = 320.0
        self.lost_frames = 0
        self.spatial_mask = None

        # ---------------- ROS 2 Pub/Sub ----------------
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            sensor_qos
        )
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.debug_pub = self.create_publisher(Image, '/camera/debug_image', 5)

        self.get_logger().info(
            'Autonomous Track Driver initialized (z=0.17m)! Subscribed to /camera/image_raw -> /cmd_vel'
        )

    def _build_spatial_mask(self, h: int, w: int) -> np.ndarray:
        """
        Builds a static 2D mask for 640x480 frames (calibrated for camera height z = 0.17m):
          1. Excludes sky and distant background above v = 259
          2. Excludes steerable front-left and front-right wheels (u < 140, u > 500)
          3. Excludes the car's red nose trapezoid (v >= 332 in center) and front rod (v >= 403)
        """
        mask = np.zeros((h, w), dtype=np.uint8)

        # Primary unobstructed lookahead band (v = 259..332, u = 140..500)
        v_top = int(0.540 * h)      # row 259 (approx 4.6m ahead of front axle)
        v_nose_tip = int(0.692 * h) # row 332 (just above nose tip for z=0.17m)
        u_left = int(0.218 * w)     # col 140 (inside left wheel)
        u_right = int(0.781 * w)    # col 500 (inside right wheel)
        mask[v_top:v_nose_tip, u_left:u_right] = 255

        # Peripheral side windows beside the nose (v = 332..403) for tight hairpin visibility
        v_rod = int(0.840 * h)      # row 403 (just above black front rod for z=0.17m)
        mask[v_nose_tip:v_rod, int(0.240 * w):int(0.395 * w)] = 255  # left of nose
        mask[v_nose_tip:v_rod, int(0.605 * w):int(0.760 * w)] = 255  # right of nose

        return mask

    @staticmethod
    def _ros_image_to_bgr(msg: Image) -> np.ndarray:
        """Converts sensor_msgs/msg/Image (rgb8 or bgr8) to an OpenCV BGR uint8 array."""
        dtype = np.uint8
        img_buf = np.frombuffer(msg.data, dtype=dtype)
        frame = img_buf.reshape((msg.height, msg.width, 3))
        if msg.encoding.lower() in ('rgb8', 'r8g8b8'):
            return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        return frame.copy()

    @staticmethod
    def _bgr_to_ros_image(bgr: np.ndarray, header) -> Image:
        """Converts an OpenCV BGR uint8 array to sensor_msgs/msg/Image (bgr8)."""
        msg = Image()
        msg.header = header
        msg.height, msg.width = bgr.shape[:2]
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = bgr.tobytes()
        return msg

    def _segment_track_surface(self, bgr: np.ndarray) -> np.ndarray:
        """
        Segments the drivable dark-grey asphalt and white lane markings in HSV space
        while rejecting green grass, teal water ponds, orange curbs, the red car nose,
        and pure black wheels/shadows.
        """
        h, w = bgr.shape[:2]
        if self.spatial_mask is None or self.spatial_mask.shape != (h, w):
            self.spatial_mask = self._build_spatial_mask(h, w)

        blurred = cv2.GaussianBlur(bgr, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # 1. Neutral dark-grey asphalt: low saturation (S <= 58), moderate value (32 <= V <= 170)
        asphalt_mask = cv2.inRange(hsv, np.array([0, 0, 32]), np.array([180, 58, 170]))

        # 2. White lane markings (center dashed line + outer boundaries): low S <= 52, high V >= 170
        white_mask = cv2.inRange(hsv, np.array([0, 0, 170]), np.array([180, 52, 255]))

        # 3. Explicit rejection masks for grass (green), water (teal), curbs (orange), and car nose (red)
        grass_water_mask = cv2.inRange(hsv, np.array([26, 35, 25]), np.array([135, 255, 255]))
        red_orange_low = cv2.inRange(hsv, np.array([0, 75, 40]), np.array([25, 255, 255]))
        red_high = cv2.inRange(hsv, np.array([165, 75, 40]), np.array([180, 255, 255]))
        non_track_mask = cv2.bitwise_or(grass_water_mask, cv2.bitwise_or(red_orange_low, red_high))

        # Combine asphalt and white markings, then subtract any non-track chromatic bleed
        drivable = cv2.bitwise_or(asphalt_mask, white_mask)
        drivable = cv2.bitwise_and(drivable, cv2.bitwise_not(non_track_mask))

        # Apply geometric self-occlusion & horizon mask
        drivable = cv2.bitwise_and(drivable, self.spatial_mask)

        # Morphological cleanup to merge dashed centerlines with asphalt and remove speckle noise
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        drivable = cv2.morphologyEx(drivable, cv2.MORPH_CLOSE, kernel)
        drivable = cv2.morphologyEx(drivable, cv2.MORPH_OPEN, kernel)

        # Connected-component filtering: lock onto the road ribbon connected to the near field
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(drivable, connectivity=8)
        if num_labels <= 1:
            return drivable

        best_label = -1
        best_score = -1.0
        for lbl in range(1, num_labels):
            area = stats[lbl, cv2.CC_STAT_AREA]
            if area < 180:
                continue
            comp_top = stats[lbl, cv2.CC_STAT_TOP]
            comp_bot = comp_top + stats[lbl, cv2.CC_STAT_HEIGHT]
            cx, cy = centroids[lbl]

            reach_bonus = 2.0 if comp_bot >= int(0.625 * h) else 0.4
            dist_penalty = 1.0 + abs(cx - self.last_road_cx) / 160.0
            score = (area * reach_bonus) / dist_penalty
            if score > best_score:
                best_score = score
                best_label = lbl

        if best_label == -1:
            return np.zeros_like(drivable)

        locked_road = np.where(labels == best_label, 255, 0).astype(np.uint8)
        return locked_road

    def image_callback(self, msg: Image):
        try:
            bgr = self._ros_image_to_bgr(msg)
        except Exception as e:
            self.get_logger().error(f'Failed to decode camera frame: {e}')
            return

        h, w = bgr.shape[:2]
        center_u = w / 2.0  # 320.0 px
        norm_half_w = 180.0 # half of active horizontal lookahead corridor (500 - 140 = 360 px)

        road_mask = self._segment_track_surface(bgr)

        # Define lookahead bands for z = 0.17m:
        # Far Band:  v = 259..295 (1.51m to 4.60m ahead of front axle)
        # Near Band: v = 295..332 (0.80m to 1.51m ahead of front axle)
        v_far_top = int(0.540 * h)   # 259
        v_mid = int(0.615 * h)       # 295
        v_near_bot = int(0.692 * h)  # 332
        v_side_bot = int(0.840 * h)  # 403

        far_band = road_mask[v_far_top:v_mid, :]
        near_band = road_mask[v_mid:v_near_bot, :]
        side_band = road_mask[v_near_bot:v_side_bot, :]

        M_near = cv2.moments(near_band)
        M_far = cv2.moments(far_band)
        M_side = cv2.moments(side_band)

        cx_near = (M_near['m10'] / M_near['m00']) if M_near['m00'] > 400.0 else None
        cx_far = (M_far['m10'] / M_far['m00']) if M_far['m00'] > 400.0 else None
        cx_side = (M_side['m10'] / M_side['m00']) if M_side['m00'] > 300.0 else None

        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds * 1e-9
        if dt <= 0.001 or dt > 0.5:
            dt = 0.1
        self.prev_time = now

        cmd = Twist()

        # ---------------- Control Law & Apex Recovery ----------------
        if cx_near is None and cx_far is None and cx_side is None:
            self.lost_frames += 1
            recovery_steer = math.copysign(self.max_steer, self.last_valid_steer)
            if abs(self.last_valid_steer) < 0.05:
                recovery_steer = 0.35
            v_cmd = self.search_speed
            omega_cmd = (v_cmd / self.wheel_base) * math.tan(recovery_steer)
            cmd.linear.x = float(v_cmd)
            cmd.angular.z = float(omega_cmd)
            self.cmd_pub.publish(cmd)
            self._publish_debug_hud(bgr, road_mask, None, None, recovery_steer, v_cmd, omega_cmd, msg.header)
            return

        self.lost_frames = 0

        if cx_near is None and cx_far is not None:
            cx_near = cx_far
        elif cx_far is None and cx_near is not None:
            cx_far = cx_near
        elif cx_near is None and cx_far is None and cx_side is not None:
            cx_near = cx_side
            cx_far = cx_side

        self.last_road_cx = 0.65 * cx_near + 0.35 * cx_far

        err_near = (cx_near - center_u) / norm_half_w
        err_far = (cx_far - center_u) / norm_half_w
        curvature_indicator = abs(err_far - err_near)

        combined_err = 0.52 * err_near + 0.48 * err_far
        d_err = (combined_err - self.prev_error) / dt
        self.prev_error = combined_err

        raw_steer = -(self.kp_near * err_near + self.kp_far * err_far + self.kd_steer * d_err)
        raw_steer = max(-self.max_steer, min(self.max_steer, raw_steer))

        self.smoothed_steer = (
            self.alpha_steer * raw_steer + (1.0 - self.alpha_steer) * self.smoothed_steer
        )
        self.last_valid_steer = self.smoothed_steer

        turn_severity = min(
            1.0,
            0.75 * (abs(self.smoothed_steer) / self.max_steer) + 0.65 * curvature_indicator
        )
        v_cmd = self.max_speed - (self.max_speed - self.min_speed) * turn_severity

        omega_cmd = (v_cmd / self.wheel_base) * math.tan(self.smoothed_steer)

        cmd.linear.x = float(v_cmd)
        cmd.angular.z = float(omega_cmd)
        self.cmd_pub.publish(cmd)

        self._publish_debug_hud(
            bgr, road_mask, cx_near, cx_far, self.smoothed_steer, v_cmd, omega_cmd, msg.header
        )

    def _publish_debug_hud(
        self, bgr: np.ndarray, road_mask: np.ndarray,
        cx_near, cx_far, steer_rad: float, v_cmd: float, omega_cmd: float, header
    ):
        """Overlays perception bands, centroids, and control telemetry onto a debug image."""
        if self.debug_pub.get_subscription_count() == 0 and not self.show_cv_window:
            return

        hud = bgr.copy()
        h, w = hud.shape[:2]

        green_overlay = np.zeros_like(hud)
        green_overlay[:, :, 1] = road_mask
        hud = cv2.addWeighted(hud, 0.78, green_overlay, 0.22, 0)

        cv2.rectangle(hud, (140, int(0.540 * h)), (500, int(0.615 * h)), (232, 115, 26), 1)
        cv2.rectangle(hud, (140, int(0.615 * h)), (500, int(0.692 * h)), (203, 181, 18), 1)
        cv2.line(hud, (w // 2, int(0.50 * h)), (w // 2, int(0.72 * h)), (200, 200, 200), 1)

        if cx_near is not None and cx_far is not None:
            pt_near = (int(cx_near), int(0.653 * h))
            pt_far = (int(cx_far), int(0.577 * h))
            cv2.line(hud, (w // 2, int(0.71 * h)), pt_near, (0, 255, 255), 2)
            cv2.line(hud, pt_near, pt_far, (0, 165, 255), 2)
            cv2.circle(hud, pt_near, 6, (0, 255, 0), -1)
            cv2.circle(hud, pt_far, 6, (0, 140, 255), -1)

        status = 'TRACKING' if cx_near is not None else f'APEX RECOVERY ({self.lost_frames})'
        cv2.rectangle(hud, (10, 10), (390, 95), (20, 20, 20), -1)
        cv2.putText(hud, f'MODE:  {status}', (20, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 128), 2)
        cv2.putText(
            hud, f'SPEED: {v_cmd:.2f} m/s | STEER: {math.degrees(steer_rad):+5.1f} deg',
            (20, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 1
        )
        cv2.putText(
            hud, f'CMD_VEL angular.z: {omega_cmd:+.3f} rad/s',
            (20, 82), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (200, 220, 255), 1
        )

        if self.debug_pub.get_subscription_count() > 0:
            self.debug_pub.publish(self._bgr_to_ros_image(hud, header))

        if self.show_cv_window:
            cv2.imshow('Project_CALIDAS Autonomous Camera View', hud)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousTrackDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_msg = Twist()
        node.cmd_pub.publish(stop_msg)
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()