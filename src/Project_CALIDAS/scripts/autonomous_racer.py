#!/usr/bin/env python3
"""
Sub-Project 2 (LI): Autonomous 3D Track LiDAR Racer for Project_CALIDAS
Target Environment: Ubuntu 24.04 | ROS 2 Jazzy | Gazebo Harmonic
Input:  /scan (sensor_msgs/msg/LaserScan, 360 rays, -3.14 to +3.14 rad @ 10 Hz)
Output: /cmd_vel (geometry_msgs/msg/Twist, Ackermann kinematically coupled)
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class AutonomousLidarRacer(Node):
    def __init__(self):
        super().__init__('autonomous_lidar_racer')

        # ---------------- Tunable Parameters ----------------
        self.declare_parameter('wheel_base', 0.76)          # meters (Ackermann wheelbase)
        self.declare_parameter('max_steer_angle', 0.58)     # radians (~33.2 deg, matches URDF 0.6 limit)
        self.declare_parameter('max_speed', 2.00)           # m/s on open straights
        self.declare_parameter('min_speed', 0.75)           # m/s in tight hairpins
        self.declare_parameter('reverse_speed', -0.50)      # m/s during close-wall unstick recovery
        self.declare_parameter('safety_stop_dist', 0.65)    # meters front clearance trigger for reverse
        self.declare_parameter('max_lidar_range', 10.0)     # meters cap for gap horizon
        self.declare_parameter('k_gap', 0.65)               # gain on Follow-the-Gap target angle
        self.declare_parameter('k_wall', 0.45)              # gain on left-right wall centering
        self.declare_parameter('kd_steer', 0.10)            # derivative damping gain
        self.declare_parameter('steering_smoothing', 0.65)  # EMA low-pass filter weight (0..1)

        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.max_steer = float(self.get_parameter('max_steer_angle').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.min_speed = float(self.get_parameter('min_speed').value)
        self.reverse_speed = float(self.get_parameter('reverse_speed').value)
        self.safety_dist = float(self.get_parameter('safety_stop_dist').value)
        self.max_range = float(self.get_parameter('max_lidar_range').value)
        self.k_gap = float(self.get_parameter('k_gap').value)
        self.k_wall = float(self.get_parameter('k_wall').value)
        self.kd_steer = float(self.get_parameter('kd_steer').value)
        self.alpha_steer = float(self.get_parameter('steering_smoothing').value)

        # ---------------- State Variables ----------------
        self.prev_err = 0.0
        self.prev_time = self.get_clock().now()
        self.smoothed_steer = 0.0
        self.reversing_until = 0.0
        self.reverse_steer_dir = 1.0
        self.log_counter = 0

        # ---------------- ROS 2 Pub/Sub ----------------
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            sensor_qos
        )
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info(
            'Autonomous LiDAR Racer initialized! Subscribed to /scan -> Publishing to /cmd_vel'
        )

    def scan_callback(self, msg: LaserScan):
        now = self.get_clock().now()
        now_sec = now.nanoseconds * 1e-9
        dt = (now - self.prev_time).nanoseconds * 1e-9
        if dt <= 0.001 or dt > 0.5:
            dt = 0.1
        self.prev_time = now

        # 1. Convert incoming ranges list to 1D NumPy float32 array
        raw_ranges = np.array(msg.ranges, dtype=np.float32)
        num_rays = len(raw_ranges)
        if num_rays == 0:
            return

        # 2. Construct exact angle array (in radians) for each ray index
        angles = msg.angle_min + np.arange(num_rays, dtype=np.float32) * msg.angle_increment

        # 3. Sanitize NaNs, infinities, and out-of-bound values:
        #    - NaN or < range_min (0.3m) -> 0.0m (immediate obstacle or self-hit)
        #    - +Inf or > max_range (10.0m) -> max_range (open track horizon)
        clean_ranges = np.where(np.isnan(raw_ranges), 0.0, raw_ranges)
        clean_ranges = np.where(np.isinf(clean_ranges), self.max_range, clean_ranges)
        clean_ranges = np.clip(clean_ranges, 0.0, self.max_range)

        # 4. Build boolean angular sector masks (0 rad = +X straight ahead, +rad = Left, -rad = Right)
        front_mask = (angles >= math.radians(-18.0)) & (angles <= math.radians(18.0))
        left_mask = (angles >= math.radians(25.0)) & (angles <= math.radians(85.0))
        right_mask = (angles >= math.radians(-85.0)) & (angles <= math.radians(-25.0))
        fov_mask = (angles >= math.radians(-75.0)) & (angles <= math.radians(75.0))

        # Use 20th percentile in front sector to catch narrow walls/barriers without single-ray noise
        d_front = float(np.percentile(clean_ranges[front_mask], 20)) if np.any(front_mask) else 2.0
        d_left = float(np.mean(clean_ranges[left_mask])) if np.any(left_mask) else 2.0
        d_right = float(np.mean(clean_ranges[right_mask])) if np.any(right_mask) else 2.0

        cmd = Twist()

        # 5. Ackermann Reverse-and-Unstick Recovery State Machine
        if now_sec < self.reversing_until:
            if d_front > 1.35:
                # Enough front clearance regained early -> exit reverse
                self.reversing_until = 0.0
            else:
                v_cmd = self.reverse_speed
                # Counter-steer while backing up so the nose swings toward the open side
                steer_rev = -self.reverse_steer_dir * self.max_steer
                omega_cmd = (v_cmd / self.wheel_base) * math.tan(steer_rev)
                cmd.linear.x = float(v_cmd)
                cmd.angular.z = float(omega_cmd)
                self.cmd_pub.publish(cmd)
                return

        if d_front < self.safety_dist:
            # Trigger 1.2-second reverse maneuver; remember which side is more open (+1=Left, -1=Right)
            self.reversing_until = now_sec + 1.2
            self.reverse_steer_dir = 1.0 if d_left >= d_right else -1.0
            self.get_logger().warn(
                f'Front wall at {d_front:.2f}m! Reversing for 1.2s to swing nose toward '
                f'{"LEFT" if self.reverse_steer_dir > 0 else "RIGHT"}...'
            )
            v_cmd = self.reverse_speed
            steer_rev = -self.reverse_steer_dir * self.max_steer
            cmd.linear.x = float(v_cmd)
            cmd.angular.z = float((v_cmd / self.wheel_base) * math.tan(steer_rev))
            self.cmd_pub.publish(cmd)
            return

        # 6. Follow-the-Gap (FTG) Target Finding in Forward FOV (-75 deg to +75 deg)
        fov_angles = angles[fov_mask]
        fov_ranges = clean_ranges[fov_mask].copy()

        # Apply safety bubble: zero out 5 rays on each side of the closest obstacle point in FOV
        min_idx = int(np.argmin(fov_ranges))
        if fov_ranges[min_idx] < 1.8:
            b_start = max(0, min_idx - 5)
            b_end = min(len(fov_ranges), min_idx + 6)
            fov_ranges[b_start:b_end] = 0.0

        # Smooth forward ranges with a 9-ray moving average window to find widest open corridor
        kernel = np.ones(9, dtype=np.float32) / 9.0
        smoothed_fov = np.convolve(fov_ranges, kernel, mode='same')
        best_idx = int(np.argmax(smoothed_fov))
        theta_gap = float(fov_angles[best_idx])

        # 7. Wall-Centering Error + PD Steering Blend
        wall_err = (d_left - d_right) / max(d_left + d_right, 1.0)
        combined_err = self.k_gap * theta_gap + self.k_wall * wall_err
        d_err = (combined_err - self.prev_err) / dt
        self.prev_err = combined_err

        raw_steer = combined_err + self.kd_steer * d_err
        raw_steer = max(-self.max_steer, min(self.max_steer, raw_steer))

        # Exponential Moving Average (EMA) steering filter
        self.smoothed_steer = (
            self.alpha_steer * raw_steer + (1.0 - self.alpha_steer) * self.smoothed_steer
        )

        # 8. Curvature & Clearance Adaptive Longitudinal Velocity
        turn_ratio = abs(self.smoothed_steer) / self.max_steer
        dist_ratio = min(1.0, max(0.0, (d_front - 0.80) / 3.20))
        speed_factor = 0.55 * dist_ratio + 0.45 * (1.0 - turn_ratio)
        v_cmd = self.min_speed + (self.max_speed - self.min_speed) * speed_factor

        # 9. Ackermann Kinematic Conversion: omega = (v / L) * tan(delta)
        omega_cmd = (v_cmd / self.wheel_base) * math.tan(self.smoothed_steer)

        cmd.linear.x = float(v_cmd)
        cmd.angular.z = float(omega_cmd)
        self.cmd_pub.publish(cmd)

        self.log_counter += 1
        if self.log_counter % 20 == 0:
            self.get_logger().info(
                f'Front: {d_front:.2f}m | L/R: {d_left:.2f}m/{d_right:.2f}m | '
                f'Gap: {math.degrees(theta_gap):+5.1f}deg | '
                f'Steer: {math.degrees(self.smoothed_steer):+5.1f}deg | v: {v_cmd:.2f}m/s'
            )


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousLidarRacer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_msg = Twist()
        node.cmd_pub.publish(stop_msg)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()