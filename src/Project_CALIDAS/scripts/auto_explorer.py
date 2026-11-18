#!/usr/bin/env python3
"""
Sub-Project 3 (DAS): Single-Camera Stop-and-Scan 360° Explorer for Project_CALIDAS
Cycle:
  1. SCAN_360:    Robot stops; single camera rotates 360 deg (+1.2 rad/s) to map all 36 sectors.
  2. RETURN_HOME: Camera returns to default forward position (0.0 rad) and locks.
  3. ALIGN_ROBOT: Robot turns toward the best open corridor (or backtracks out of dead ends).
  4. DRIVE_STEP:  Robot drives forward one step (1.8m) with camera watching ahead, stops, and repeats!
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, LaserScan, JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist


class SingleCameraStepExplorer(Node):
    def __init__(self):
        super().__init__('auto_city_explorer')

        # ---------------- Tunable Parameters ----------------
        self.declare_parameter('step_distance', 1.80)       # meters to drive per step
        self.declare_parameter('cruise_speed', 0.45)        # m/s forward speed during DRIVE_STEP
        self.declare_parameter('turn_speed', 0.80)          # rad/s robot rotation speed
        self.declare_parameter('camera_scan_speed', 1.20)   # rad/s camera turret speed during 360 scan
        self.declare_parameter('stop_distance', 0.85)       # meters front safety stop distance
        self.declare_parameter('open_corridor_dist', 1.75)  # meters required to consider a path open
        self.declare_parameter('min_valid_distance', 0.45)  # meters self-chassis hit filter
        self.declare_parameter('max_depth_range', 8.0)      # meters max depth ceiling

        self.step_dist = float(self.get_parameter('step_distance').value)
        self.cruise_speed = float(self.get_parameter('cruise_speed').value)
        self.turn_speed = float(self.get_parameter('turn_speed').value)
        self.cam_scan_speed = float(self.get_parameter('camera_scan_speed').value)
        self.stop_dist = float(self.get_parameter('stop_distance').value)
        self.open_dist = float(self.get_parameter('open_corridor_dist').value)
        self.min_valid = float(self.get_parameter('min_valid_distance').value)
        self.max_depth = float(self.get_parameter('max_depth_range').value)

        # ---------------- 360-Degree Polar Map (36 bins x 10 deg) ----------------
        self.num_bins = 36
        self.bin_centers_rad = np.linspace(
            -math.pi + (math.pi / 36.0),
            math.pi - (math.pi / 36.0),
            self.num_bins,
            dtype=np.float32
        )
        self.bin_centers_deg = np.degrees(self.bin_centers_rad)
        self.polar_ring = np.full(self.num_bins, self.max_depth, dtype=np.float32)
        self.front_clearance = self.max_depth

        # ---------------- Camera & Robot State ----------------
        self.camera_yaw = 0.0           # Wrapped to [-pi, +pi], 0.0 = default straight ahead
        self.last_cam_yaw = None
        self.accumulated_scan_rad = 0.0 # Tracks progress of the 360-degree (2*pi) sweep
        self.joint_state_received = False

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.step_start_x = 0.0
        self.step_start_y = 0.0
        self.target_robot_yaw = 0.0

        # States: 'SCAN_360' -> 'RETURN_HOME' -> 'ALIGN_ROBOT' -> 'DRIVE_STEP'
        self.state = 'SCAN_360'
        self.state_start_time = 0.0
        self.log_counter = 0

        # ---------------- ROS 2 Pub/Sub ----------------
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth_image', self.depth_callback, sensor_qos
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_fallback_callback, sensor_qos
        )
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cam_vel_pub = self.create_publisher(Float64, '/camera_joint/cmd_vel', 10)
        self.timer = self.create_timer(0.10, self.control_loop)

        self.get_logger().info(
            'Single-Camera Stop-and-Scan Explorer started! Starting initial 360-degree camera scan...'
        )

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def _publish_cam_vel(self, omega: float):
        msg = Float64()
        msg.data = float(omega)
        self.cam_vel_pub.publish(msg)

    def joint_callback(self, msg: JointState):
        if 'camera_joint' in msg.name:
            idx = msg.name.index('camera_joint')
            if idx < len(msg.position):
                current_yaw = self._wrap_angle(msg.position[idx])
                if self.last_cam_yaw is not None and self.state == 'SCAN_360':
                    delta = abs(self._wrap_angle(current_yaw - self.last_cam_yaw))
                    if delta < 1.0:
                        self.accumulated_scan_rad += delta
                self.last_cam_yaw = current_yaw
                self.camera_yaw = current_yaw
                self.joint_state_received = True

    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def _decode_depth(self, msg: Image) -> np.ndarray:
        enc = msg.encoding.lower()
        if '32fc1' in enc:
            return np.frombuffer(msg.data, dtype=np.float32).reshape((msg.height, msg.width))
        elif '16uc1' in enc or 'mono16' in enc:
            raw_mm = np.frombuffer(msg.data, dtype=np.uint16).reshape((msg.height, msg.width))
            return raw_mm.astype(np.float32) * 0.001
        raw_u8 = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, -1))[:, :, 0]
        return (raw_u8.astype(np.float32) / 255.0) * self.max_depth

    def depth_callback(self, msg: Image):
        try:
            depth = self._decode_depth(msg)
        except Exception:
            return

        h, w = depth.shape[:2]
        # Slice Horizon Obstacle Band (rows 22% to 48%) to reject flat street pavement
        roi = depth[int(0.22 * h):int(0.48 * h), :]
        clean = np.where(
            np.isnan(roi) | np.isinf(roi) | (roi > self.max_depth) | (roi < self.min_valid),
            self.max_depth,
            roi
        )
        col_depths = np.percentile(clean, 20, axis=0)

        # If camera is near home position (0.0 rad), update live forward obstacle clearance
        if abs(self.camera_yaw) < 0.30:
            center_cols = col_depths[int(0.25 * w):int(0.75 * w)]
            self.front_clearance = float(np.percentile(center_cols, 15))

        # Project each column into the 360-degree polar ring using current camera_yaw
        hfov = 1.089
        fx = (w / 2.0) / math.tan(hfov / 2.0)
        u_cols = np.arange(w, dtype=np.float32)
        local_angles = np.arctan2((w / 2.0) - u_cols, fx)

        rel_angles = np.arctan2(
            np.sin(self.camera_yaw + local_angles),
            np.cos(self.camera_yaw + local_angles)
        )
        bin_indices = np.clip(
            ((rel_angles + math.pi) / (2.0 * math.pi) * self.num_bins).astype(int),
            0,
            self.num_bins - 1
        )
        for b in np.unique(bin_indices):
            self.polar_ring[b] = float(np.min(col_depths[bin_indices == b]))

    def scan_fallback_callback(self, msg: LaserScan):
        """Provides backup safety clearance if depth image is still initializing."""
        raw = np.array(msg.ranges, dtype=np.float32)
        if len(raw) == 0:
            return
        angles = msg.angle_min + np.arange(len(raw), dtype=np.float32) * msg.angle_increment
        clean = np.where(
            np.isnan(raw) | np.isinf(raw) | (raw < self.min_valid),
            self.max_depth,
            np.minimum(raw, self.max_depth)
        )
        front_mask = (angles >= math.radians(-20.0)) & (angles <= math.radians(20.0))
        if np.any(front_mask):
            self.front_clearance = min(self.front_clearance, float(np.percentile(clean[front_mask], 20)))

    def _choose_next_heading_from_360_scan(self) -> float:
        """
        Analyzes the completed 360-degree polar_ring to pick the best direction to drive:
          1. Smooths the 36 bins with a circular moving average.
          2. If straight ahead (-20..+20 deg) is clear (> open_dist), picks the best forward angle.
          3. Otherwise, picks the deepest open gap in the forward/side hemisphere (-135..+135 deg).
          4. If boxed into a dead end (all forward/side < 1.25m), picks the deepest rear exit (~180 deg).
        """
        padded = np.pad(self.polar_ring, 2, mode='wrap')
        smoothed = np.convolve(padded, np.ones(5, dtype=np.float32) / 5.0, mode='valid')

        fwd_mask = (self.bin_centers_deg >= -25.0) & (self.bin_centers_deg <= 25.0)
        if float(np.min(self.polar_ring[fwd_mask])) >= self.open_dist:
            best_fwd_idx = int(np.argmax(smoothed[fwd_mask]))
            rel_angle = float(self.bin_centers_rad[fwd_mask][best_fwd_idx])
            return self._wrap_angle(self.robot_yaw + 0.5 * rel_angle)

        # Front is obstructed -> check forward and side gaps (-135 deg to +135 deg)
        front_side_mask = (self.bin_centers_deg >= -135.0) & (self.bin_centers_deg <= 135.0)
        if float(np.max(smoothed[front_side_mask])) >= 1.25:
            best_side_idx = int(np.argmax(smoothed[front_side_mask]))
            rel_angle = float(self.bin_centers_rad[front_side_mask][best_side_idx])
            self.get_logger().info(
                f'Front blocked! Selecting open gap at {math.degrees(rel_angle):+.0f} deg from 360 scan.'
            )
            return self._wrap_angle(self.robot_yaw + rel_angle)

        # Dead end! All front & side paths blocked -> turn around toward deepest rear gap
        best_any_idx = int(np.argmax(smoothed))
        rel_angle = float(self.bin_centers_rad[best_any_idx])
        self.get_logger().warn(
            f'Dead end detected! Turning back toward rear gap at {math.degrees(rel_angle):+.0f} deg.'
        )
        return self._wrap_angle(self.robot_yaw + rel_angle)

    def control_loop(self):
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if self.state_start_time == 0.0:
            self.state_start_time = now_sec

        elapsed = now_sec - self.state_start_time
        cmd = Twist()

        # ==================== STAGE 1: SCAN_360 ====================
        if self.state == 'SCAN_360':
            # Keep robot base completely stationary while camera rotates 360 deg
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            self._publish_cam_vel(self.cam_scan_speed)

            # Complete when camera has rotated 2*pi rad (or fallback timer if joint_states lags)
            scan_time_target = (2.0 * math.pi) / max(self.cam_scan_speed, 0.2)
            if self.accumulated_scan_rad >= (2.0 * math.pi) or (elapsed > scan_time_target + 1.0):
                self.state = 'RETURN_HOME'
                self.state_start_time = now_sec
                self.get_logger().info(
                    '360-degree camera scan complete! Returning camera to default 0.0 rad position...'
                )
            return

        # ==================== STAGE 2: RETURN_HOME ====================
        if self.state == 'RETURN_HOME':
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)

            # Proportional controller to bring camera_yaw back to 0.0 rad (default forward pose)
            if abs(self.camera_yaw) <= 0.05 or elapsed > 4.0:
                self._publish_cam_vel(0.0)  # Lock camera at default forward position
                self.target_robot_yaw = self._choose_next_heading_from_360_scan()
                self.state = 'ALIGN_ROBOT'
                self.state_start_time = now_sec
                self.get_logger().info(
                    f'Camera locked at default (0.0 rad). Aligning robot to heading '
                    f'{math.degrees(self.target_robot_yaw):+.1f} deg...'
                )
            else:
                home_vel = max(-1.5, min(1.5, -2.5 * self.camera_yaw))
                if abs(home_vel) < 0.20:
                    home_vel = math.copysign(0.20, home_vel)
                self._publish_cam_vel(home_vel)
            return

        # ==================== STAGE 3: ALIGN_ROBOT ====================
        if self.state == 'ALIGN_ROBOT':
            self._publish_cam_vel(0.0)
            yaw_err = self._wrap_angle(self.target_robot_yaw - self.robot_yaw)

            if abs(yaw_err) <= 0.08 or elapsed > 5.0:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.cmd_pub.publish(cmd)
                self.step_start_x = self.robot_x
                self.step_start_y = self.robot_y
                self.front_clearance = self.max_depth
                self.state = 'DRIVE_STEP'
                self.state_start_time = now_sec
                self.get_logger().info(
                    f'Robot aligned! Driving forward {self.step_dist:.1f}m with camera facing ahead...'
                )
            else:
                cmd.linear.x = 0.0
                rot_vel = max(-self.turn_speed, min(self.turn_speed, 1.8 * yaw_err))
                if abs(rot_vel) < 0.25:
                    rot_vel = math.copysign(0.25, rot_vel)
                cmd.angular.z = float(rot_vel)
                self.cmd_pub.publish(cmd)
            return

        # ==================== STAGE 4: DRIVE_STEP ====================
        if self.state == 'DRIVE_STEP':
            # Keep camera locked at 0.0 rad while driving forward
            home_corr = max(-0.5, min(0.5, -2.0 * self.camera_yaw)) if abs(self.camera_yaw) > 0.03 else 0.0
            self._publish_cam_vel(home_corr)

            dist_traveled = math.hypot(
                self.robot_x - self.step_start_x,
                self.robot_y - self.step_start_y
            )

            # Stop if step_distance (1.8m) reached OR if an obstacle appears closer than stop_distance (0.85m)
            if dist_traveled >= self.step_dist or self.front_clearance < self.stop_dist or elapsed > 8.0:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.cmd_pub.publish(cmd)

                reason = (
                    f'completed {dist_traveled:.2f}m step'
                    if dist_traveled >= self.step_dist
                    else f'obstacle ahead at {self.front_clearance:.2f}m'
                )
                self.get_logger().info(
                    f'Stopped ({reason}). Starting next 360-degree camera scan...'
                )

                # Reset 360 scan accumulators and start Stage 1 again
                self.accumulated_scan_rad = 0.0
                self.polar_ring.fill(self.max_depth)
                self.state = 'SCAN_360'
                self.state_start_time = now_sec
                return

            # Drive straight forward along target_robot_yaw
            heading_hold_err = self._wrap_angle(self.target_robot_yaw - self.robot_yaw)
            cmd.linear.x = float(self.cruise_speed)
            cmd.angular.z = float(0.8 * heading_hold_err)
            self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SingleCameraStepExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.cmd_pub.publish(Twist())
            node._publish_cam_vel(0.0)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()