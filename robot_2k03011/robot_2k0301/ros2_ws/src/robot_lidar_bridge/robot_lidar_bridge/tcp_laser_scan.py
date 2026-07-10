import json
import math
import socket
import threading
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan

from robot_lidar_bridge.board_clock import BoardClockMapper
from robot_lidar_bridge.lidar_deskew import deskew_point


def raw_angle_to_ros_deg(angle_deg):
    """Convert the LD19 clockwise angle to ROS counter-clockwise degrees."""
    return (-angle_deg) % 360.0


class TcpLaserScan(Node):
    def __init__(self):
        super().__init__('robot_lidar_tcp_bridge')
        self.declare_parameter('host', '192.168.123.70')
        self.declare_parameter('port', 2368)
        self.declare_parameter('frame_id', 'base_laser')
        self.declare_parameter('bins', 720)
        self.declare_parameter('range_min', 0.02)
        self.declare_parameter('range_max', 12.0)
        self.declare_parameter('transport_delay_sec', 0.0)
        self.declare_parameter('socket_timeout_sec', 2.0)
        self.declare_parameter('reconnect_delay_sec', 1.0)
        self.declare_parameter('min_scan_hz', 5.0)
        self.declare_parameter('max_scan_hz', 15.0)
        self.declare_parameter('min_valid_points', 30)
        self.declare_parameter('stats_period_sec', 5.0)
        self.declare_parameter('deskew_enabled', True)
        self.declare_parameter('deskew_odom_timeout_sec', 0.25)
        self.declare_parameter('laser_x', 0.12)
        self.declare_parameter('laser_y', 0.0)

        self.host = str(self.get_parameter('host').value)
        self.port = int(self.get_parameter('port').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.bins = max(int(self.get_parameter('bins').value), 90)
        self.range_min = max(
            float(self.get_parameter('range_min').value), 0.001)
        self.range_max = max(
            float(self.get_parameter('range_max').value), self.range_min + 0.1)
        self.transport_delay_sec = max(
            float(self.get_parameter('transport_delay_sec').value), 0.0)
        self.socket_timeout_sec = max(
            float(self.get_parameter('socket_timeout_sec').value), 0.2)
        self.reconnect_delay_sec = max(
            float(self.get_parameter('reconnect_delay_sec').value), 0.1)
        self.min_scan_hz = max(
            float(self.get_parameter('min_scan_hz').value), 0.1)
        self.max_scan_hz = max(
            float(self.get_parameter('max_scan_hz').value), self.min_scan_hz)
        self.min_valid_points = max(
            int(self.get_parameter('min_valid_points').value), 1)
        self.stats_period_sec = max(
            float(self.get_parameter('stats_period_sec').value), 1.0)
        self.deskew_enabled = bool(
            self.get_parameter('deskew_enabled').value)
        self.deskew_odom_timeout_sec = max(
            float(self.get_parameter('deskew_odom_timeout_sec').value), 0.05)
        self.laser_x = float(self.get_parameter('laser_x').value)
        self.laser_y = float(self.get_parameter('laser_y').value)

        self.publisher = self.create_publisher(LaserScan, 'scan', 10)
        self.odom_subscription = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.board_clock = BoardClockMapper()
        self.odom_linear_velocity = 0.0
        self.odom_angular_velocity = 0.0
        self.last_odom_monotonic = -math.inf
        self.running = True
        self.last_sequence = None
        self.last_warning_time = 0.0
        self.last_stats_time = time.monotonic()
        self.published_scans = 0
        self.rejected_scans = 0
        self.worker = threading.Thread(target=self.receive_loop, daemon=True)
        self.worker.start()

    def odom_callback(self, message):
        self.odom_linear_velocity = float(message.twist.twist.linear.x)
        self.odom_angular_velocity = float(message.twist.twist.angular.z)
        self.last_odom_monotonic = time.monotonic()

    def warning_limited(self, message):
        now = time.monotonic()
        if now - self.last_warning_time >= 1.0:
            self.get_logger().warning(message)
            self.last_warning_time = now

    def receive_loop(self):
        while self.running and rclpy.ok():
            try:
                self.get_logger().info(
                    f'connecting to lidar stream {self.host}:{self.port}')
                with socket.create_connection(
                        (self.host, self.port), timeout=5.0) as sock:
                    sock.settimeout(self.socket_timeout_sec)
                    self.last_sequence = None
                    self.get_logger().info('lidar stream connected')
                    with sock.makefile('r', encoding='ascii') as stream:
                        for line in stream:
                            if not self.running:
                                return
                            self.publish_scan(json.loads(line))
            except (
                    OSError, ValueError, TypeError,
                    json.JSONDecodeError) as exc:
                self.warning_limited(f'lidar stream unavailable: {exc}')
                time.sleep(self.reconnect_delay_sec)

    def validate_sequence(self, packet):
        sequence = int(packet.get('seq', -1))
        if sequence < 0:
            raise ValueError('missing lidar sequence')
        if self.last_sequence is not None and sequence <= self.last_sequence:
            raise ValueError(
                f'non-increasing lidar sequence '
                f'{self.last_sequence} -> {sequence}')
        self.last_sequence = sequence

    def publish_scan(self, packet):
        try:
            if not isinstance(packet, dict):
                raise ValueError('lidar packet is not an object')
            self.validate_sequence(packet)
            scan_hz = float(packet.get('hz', 0.0))
            if (not math.isfinite(scan_hz) or
                    not self.min_scan_hz <= scan_hz <= self.max_scan_hz):
                raise ValueError(f'invalid lidar frequency {scan_hz}')
            points = packet.get('points')
            if not isinstance(points, list):
                raise ValueError('lidar points is not a list')

            scan_time = 1.0 / scan_hz
            message = LaserScan()
            board_scan_end_ns = int(packet.get('mono_ns', 0))
            receive_time = self.get_clock().now()
            scan_midpoint_ns = board_scan_end_ns - int(scan_time * 0.5e9)
            scan_midpoint_ns -= int(self.transport_delay_sec * 1.0e9)
            stamp_ns = self.board_clock.map_ns(
                scan_midpoint_ns,
                receive_time.nanoseconds,
                sync_board_mono_ns=board_scan_end_ns)
            message.header.stamp = Time(nanoseconds=stamp_ns).to_msg()
            message.header.frame_id = self.frame_id
            message.angle_min = 0.0
            message.angle_increment = 2.0 * math.pi / self.bins
            message.angle_max = (
                message.angle_min + (self.bins - 1) * message.angle_increment)
            message.scan_time = scan_time
            # The LD19 spins clockwise while LaserScan bins increase
            # counter-clockwise after conversion. Individual point timestamps
            # are unavailable, so publishing a false positive time increment
            # would deskew in the wrong direction. Timestamp the scan midpoint
            # and treat the bins as simultaneous instead.
            message.time_increment = 0.0
            message.range_min = self.range_min
            message.range_max = self.range_max
            message.ranges = [math.inf] * self.bins
            message.intensities = [0.0] * self.bins

            valid_bins = 0
            now_monotonic = time.monotonic()
            apply_deskew = (
                self.deskew_enabled and
                now_monotonic - self.last_odom_monotonic <=
                self.deskew_odom_timeout_sec)
            linear_velocity = self.odom_linear_velocity
            angular_velocity = self.odom_angular_velocity
            for point in points:
                if not isinstance(point, list) or len(point) < 2:
                    continue
                angle_deg = float(point[0])
                distance_mm = float(point[1])
                intensity = float(point[2]) if len(point) > 2 else 0.0
                if not (math.isfinite(angle_deg) and
                        math.isfinite(distance_mm) and
                        math.isfinite(intensity)):
                    continue
                distance_m = distance_mm / 1000.0
                if not self.range_min <= distance_m <= self.range_max:
                    continue
                ros_angle_deg = raw_angle_to_ros_deg(angle_deg)
                if apply_deskew:
                    sample_offset_sec = (
                        (angle_deg % 360.0) / 360.0 - 0.5) * scan_time
                    ros_angle_deg, distance_m = deskew_point(
                        ros_angle_deg,
                        distance_m,
                        sample_offset_sec,
                        linear_velocity,
                        angular_velocity,
                        self.laser_x,
                        self.laser_y)
                index = int(ros_angle_deg / 360.0 * self.bins) % self.bins
                if distance_m < message.ranges[index]:
                    if math.isinf(message.ranges[index]):
                        valid_bins += 1
                    message.ranges[index] = distance_m
                    message.intensities[index] = intensity

            if valid_bins < self.min_valid_points:
                raise ValueError(
                    f'only {valid_bins} valid lidar bins, '
                    f'minimum is {self.min_valid_points}')
        except (ValueError, TypeError, OverflowError) as exc:
            self.rejected_scans += 1
            self.warning_limited(f'rejected lidar packet: {exc}')
            return

        self.publisher.publish(message)
        self.published_scans += 1
        now = time.monotonic()
        if now - self.last_stats_time >= self.stats_period_sec:
            self.get_logger().info(
                f'lidar healthy: published={self.published_scans} '
                f'rejected={self.rejected_scans} '
                f'hz={scan_hz:.2f} valid_bins={valid_bins}')
            self.last_stats_time = now

    def destroy_node(self):
        self.running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TcpLaserScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    main()
