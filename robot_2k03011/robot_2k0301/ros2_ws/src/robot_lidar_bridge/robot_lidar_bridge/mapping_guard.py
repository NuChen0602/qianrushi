import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


class MappingGuard(Node):
    def __init__(self):
        super().__init__('mapping_guard')
        self.declare_parameter('timeout_sec', 20.0)
        self.declare_parameter('min_scan_messages', 5)
        self.declare_parameter('min_odom_messages', 10)
        self.declare_parameter('min_finite_scan_points', 30)
        self.declare_parameter('max_message_age_sec', 1.0)
        self.declare_parameter('max_linear_speed_mps', 3.0)
        self.declare_parameter('max_angular_speed_rps', 8.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('laser_frame', 'base_laser')

        self.timeout_sec = max(
            float(self.get_parameter('timeout_sec').value), 1.0)
        self.min_scan_messages = max(
            int(self.get_parameter('min_scan_messages').value), 1)
        self.min_odom_messages = max(
            int(self.get_parameter('min_odom_messages').value), 1)
        self.min_finite_scan_points = max(
            int(self.get_parameter('min_finite_scan_points').value), 1)
        self.max_message_age_sec = max(
            float(self.get_parameter('max_message_age_sec').value), 0.2)
        self.max_linear_speed_mps = max(
            float(self.get_parameter('max_linear_speed_mps').value), 0.1)
        self.max_angular_speed_rps = max(
            float(self.get_parameter('max_angular_speed_rps').value), 0.1)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.laser_frame = str(self.get_parameter('laser_frame').value)

        self.scan_count = 0
        self.odom_count = 0
        self.invalid_scan_count = 0
        self.invalid_odom_count = 0
        self.last_scan_time = None
        self.last_odom_time = None
        self.last_status_time = 0.0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 20)

    def scan_callback(self, message):
        valid = True
        if message.header.frame_id != self.laser_frame:
            valid = False
        if (not math.isfinite(message.angle_increment) or
                message.angle_increment <= 0.0 or not message.ranges):
            valid = False
        else:
            expected = int(round(
                (message.angle_max - message.angle_min) /
                message.angle_increment)) + 1
            if abs(expected - len(message.ranges)) > 1:
                valid = False
        finite_points = sum(
            1 for value in message.ranges
            if math.isfinite(value) and
            message.range_min <= value <= message.range_max)
        if finite_points < self.min_finite_scan_points:
            valid = False

        if valid:
            self.scan_count += 1
            self.last_scan_time = time.monotonic()
        else:
            self.invalid_scan_count += 1

    def odom_callback(self, message):
        values = (
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            message.pose.pose.orientation.x,
            message.pose.pose.orientation.y,
            message.pose.pose.orientation.z,
            message.pose.pose.orientation.w,
            message.twist.twist.linear.x,
            message.twist.twist.angular.z,
        )
        quaternion_norm = math.sqrt(sum(value * value for value in values[2:6]))
        valid = (
            message.header.frame_id == self.odom_frame and
            message.child_frame_id == self.base_frame and
            all(math.isfinite(value) for value in values) and
            abs(quaternion_norm - 1.0) <= 0.05 and
            abs(message.twist.twist.linear.x) <= self.max_linear_speed_mps and
            abs(message.twist.twist.angular.z) <= self.max_angular_speed_rps
        )
        if valid:
            self.odom_count += 1
            self.last_odom_time = time.monotonic()
        else:
            self.invalid_odom_count += 1

    def transforms_ready(self):
        try:
            self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.1),
            )
            self.tf_buffer.lookup_transform(
                self.base_frame,
                self.laser_frame,
                Time(),
                timeout=Duration(seconds=0.1),
            )
            return True
        except TransformException:
            return False

    def ready(self):
        now = time.monotonic()
        scan_fresh = (
            self.last_scan_time is not None and
            now - self.last_scan_time <= self.max_message_age_sec)
        odom_fresh = (
            self.last_odom_time is not None and
            now - self.last_odom_time <= self.max_message_age_sec)
        return (
            self.scan_count >= self.min_scan_messages and
            self.odom_count >= self.min_odom_messages and
            scan_fresh and
            odom_fresh and
            self.transforms_ready()
        )

    def log_status(self):
        now = time.monotonic()
        if now - self.last_status_time < 2.0:
            return
        self.get_logger().info(
            f'waiting for healthy mapping inputs: '
            f'scan={self.scan_count}/{self.min_scan_messages} '
            f'odom={self.odom_count}/{self.min_odom_messages} '
            f'invalid_scan={self.invalid_scan_count} '
            f'invalid_odom={self.invalid_odom_count} '
            f'tf_ready={self.transforms_ready()}')
        self.last_status_time = now


def main(args=None):
    rclpy.init(args=args)
    node = MappingGuard()
    deadline = time.monotonic() + node.timeout_sec
    result = 1
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.ready():
                node.get_logger().info(
                    'mapping inputs healthy: /scan, /odom and TF are ready')
                result = 0
                break
            node.log_status()
        if result != 0:
            node.get_logger().error(
                'mapping input health check timed out; SLAM will not start')
    except KeyboardInterrupt:
        node.get_logger().warning('mapping input health check interrupted')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return result


def watchdog_main(args=None):
    rclpy.init(args=args)
    node = MappingGuard()
    startup_deadline = time.monotonic() + node.timeout_sec
    unhealthy_since = None
    healthy_once = False
    result = 0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if node.ready():
                if not healthy_once:
                    node.get_logger().info(
                        'mapping watchdog active: inputs are healthy')
                elif unhealthy_since is not None:
                    node.get_logger().info(
                        'mapping inputs recovered before watchdog timeout')
                healthy_once = True
                unhealthy_since = None
                continue

            if not healthy_once:
                node.log_status()
                if now >= startup_deadline:
                    node.get_logger().error(
                        'mapping watchdog startup timed out')
                    result = 1
                    break
                continue

            if unhealthy_since is None:
                unhealthy_since = now
                node.get_logger().warning(
                    'mapping inputs became unhealthy; waiting 2 seconds '
                    'before stopping')
            elif now - unhealthy_since >= 2.0:
                node.get_logger().error(
                    'mapping inputs remained unhealthy; stopping this run '
                    'to prevent map corruption')
                result = 1
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return result


if __name__ == '__main__':
    raise SystemExit(main())
