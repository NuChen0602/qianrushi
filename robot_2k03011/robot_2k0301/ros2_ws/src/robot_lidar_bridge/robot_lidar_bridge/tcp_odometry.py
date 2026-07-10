import json
import math
import socket
import threading
import time

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan

from robot_lidar_bridge.board_clock import BoardClockMapper
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from robot_lidar_bridge.motion_control import SteeringCalibration


class TcpOdometry(Node):
    def __init__(self):
        super().__init__('robot_odometry_tcp_bridge')
        self.declare_parameter('host', '192.168.123.70')
        self.declare_parameter('port', 2369)
        self.declare_parameter('counts_per_meter', 20000.0)
        self.declare_parameter('left_counts_per_meter', -1.0)
        self.declare_parameter('right_counts_per_meter', -1.0)
        self.declare_parameter('max_counts_per_packet', 2000)
        self.declare_parameter('gyro_z_sign', 1.0)
        self.declare_parameter('gyro_deadband_dps', 0.1)
        self.declare_parameter('socket_timeout_sec', 2.0)
        self.declare_parameter('reconnect_delay_sec', 1.0)
        self.declare_parameter('min_dt', 0.005)
        self.declare_parameter('max_dt', 0.1)
        self.declare_parameter('max_linear_speed_mps', 3.0)
        self.declare_parameter('max_angular_speed_rps', 8.0)
        self.declare_parameter('require_imu_ready', True)
        self.declare_parameter('stats_period_sec', 5.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('laser_frame', 'base_laser')
        self.declare_parameter('laser_x', 0.12)
        self.declare_parameter('laser_y', 0.0)
        self.declare_parameter('laser_yaw', math.pi / 2.0)
        self.declare_parameter('enable_drive', False)
        self.declare_parameter('enable_drive_obstacle_safety', True)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('cmd_vel_timeout_sec', 0.35)
        self.declare_parameter('scan_timeout_sec', 0.6)
        self.declare_parameter('max_drive_speed_mps', 0.38)
        self.declare_parameter('max_steering_command', 1.0)
        self.declare_parameter('servo_center_deg', 95.0)
        self.declare_parameter('servo_left_deg', 125.0)
        self.declare_parameter('servo_right_deg', 80.0)
        self.declare_parameter('wheelbase_m', 0.18)
        self.declare_parameter(
            'steering_command_points', [-1.0, -0.5, 0.0, 0.5, 1.0])
        self.declare_parameter(
            'steering_servo_deg_points', [80.0, 87.5, 95.0, 110.0, 125.0])
        self.declare_parameter(
            'steering_wheel_deg_points', [-37.0, -18.5, 0.0, 18.5, 37.0])
        self.declare_parameter('drive_status_topic', '/drive/status')
        self.declare_parameter('drive_status_period_sec', 0.1)
        self.declare_parameter('max_servo_rate_deg_per_sec', 40.0)
        self.declare_parameter('front_sector_center_rad', 0.0)
        self.declare_parameter(
            'front_sector_half_width_rad', math.radians(30.0))
        self.declare_parameter('obstacle_stop_distance_m', 0.5)
        self.declare_parameter('obstacle_slow_distance_m', 0.8)

        self.host = str(self.get_parameter('host').value)
        self.port = int(self.get_parameter('port').value)
        legacy_counts_per_meter = max(
            float(self.get_parameter('counts_per_meter').value), 1.0)
        left_counts_per_meter = float(
            self.get_parameter('left_counts_per_meter').value)
        right_counts_per_meter = float(
            self.get_parameter('right_counts_per_meter').value)
        self.left_counts_per_meter = (
            left_counts_per_meter if left_counts_per_meter > 0.0
            else legacy_counts_per_meter)
        self.right_counts_per_meter = (
            right_counts_per_meter if right_counts_per_meter > 0.0
            else legacy_counts_per_meter)
        self.max_counts_per_packet = max(
            int(self.get_parameter('max_counts_per_packet').value), 1)
        self.gyro_z_sign = float(self.get_parameter('gyro_z_sign').value)
        self.gyro_deadband_dps = abs(
            float(self.get_parameter('gyro_deadband_dps').value))
        self.socket_timeout_sec = max(
            float(self.get_parameter('socket_timeout_sec').value), 0.2)
        self.reconnect_delay_sec = max(
            float(self.get_parameter('reconnect_delay_sec').value), 0.1)
        self.min_dt = max(float(self.get_parameter('min_dt').value), 0.001)
        self.max_dt = max(
            float(self.get_parameter('max_dt').value), self.min_dt)
        self.max_linear_speed_mps = max(
            float(self.get_parameter('max_linear_speed_mps').value), 0.1)
        self.max_angular_speed_rps = max(
            float(self.get_parameter('max_angular_speed_rps').value), 0.1)
        self.require_imu_ready = bool(
            self.get_parameter('require_imu_ready').value)
        self.stats_period_sec = max(
            float(self.get_parameter('stats_period_sec').value), 1.0)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.laser_frame = str(self.get_parameter('laser_frame').value)
        self.enable_drive = bool(self.get_parameter('enable_drive').value)
        self.enable_drive_obstacle_safety = bool(
            self.get_parameter('enable_drive_obstacle_safety').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.cmd_vel_timeout_sec = max(
            float(self.get_parameter('cmd_vel_timeout_sec').value), 0.1)
        self.scan_timeout_sec = max(
            float(self.get_parameter('scan_timeout_sec').value), 0.2)
        self.max_drive_speed_mps = max(
            abs(float(self.get_parameter('max_drive_speed_mps').value)), 0.01)
        self.max_steering_command = max(
            abs(float(self.get_parameter('max_steering_command').value)), 0.01)
        self.servo_center_deg = float(
            self.get_parameter('servo_center_deg').value)
        self.servo_left_deg = float(self.get_parameter('servo_left_deg').value)
        self.servo_right_deg = float(
            self.get_parameter('servo_right_deg').value)
        self.steering_calibration = self.load_steering_calibration()
        self.drive_status_period_sec = max(
            float(self.get_parameter('drive_status_period_sec').value), 0.05)
        self.max_servo_rate_deg_per_sec = max(
            float(self.get_parameter('max_servo_rate_deg_per_sec').value),
            1.0)
        self.front_sector_center_rad = float(
            self.get_parameter('front_sector_center_rad').value)
        self.front_sector_half_width_rad = max(
            abs(float(
                self.get_parameter('front_sector_half_width_rad').value)),
            math.radians(1.0))
        self.obstacle_stop_distance_m = max(
            float(self.get_parameter('obstacle_stop_distance_m').value), 0.02)
        self.obstacle_slow_distance_m = max(
            float(self.get_parameter('obstacle_slow_distance_m').value),
            self.obstacle_stop_distance_m + 0.1)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.board_clock = BoardClockMapper()
        self.last_sequence = None
        self.first_packet_after_connect = True
        self.last_warning_time = 0.0
        self.last_stats_time = time.monotonic()
        self.published_packets = 0
        self.rejected_packets = 0
        self.command_lock = threading.Lock()
        self.command_linear = 0.0
        self.command_angular = 0.0
        self.last_command_time = 0.0
        self.front_distance_m = math.inf
        self.rear_distance_m = math.inf
        self.last_scan_time = 0.0
        self.drive_stop_reason = ''
        self.last_drive_status_time = 0.0
        self.applied_servo_deg = self.steering_calibration.servo_deg(0.0)
        self.last_servo_update_time = 0.0
        self.active_socket = None

        self.publisher = self.create_publisher(Odometry, 'odom', 20)
        self.drive_status_publisher = self.create_publisher(
            String,
            str(self.get_parameter('drive_status_topic').value),
            10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_laser_transform()

        if self.enable_drive:
            self.cmd_subscription = self.create_subscription(
                Twist, self.cmd_vel_topic, self.cmd_vel_callback, 10)
            self.scan_subscription = self.create_subscription(
                LaserScan, '/scan', self.scan_callback, 10)
            self.get_logger().info(
                f'remote drive enabled: cmd={self.cmd_vel_topic} '
                f'max_speed={self.max_drive_speed_mps:.2f}m/s '
                f'obstacle_safety={self.enable_drive_obstacle_safety} '
                f'steering_calibration_points='
                f'{len(self.steering_calibration.command_points)}')

        self.running = True
        self.worker = threading.Thread(target=self.receive_loop, daemon=True)
        self.worker.start()

    @staticmethod
    def yaw_quaternion(yaw):
        return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)

    def load_steering_calibration(self):
        command_points = tuple(float(value) for value in
                               self.get_parameter(
                                   'steering_command_points').value)
        servo_points = tuple(float(value) for value in
                             self.get_parameter(
                                 'steering_servo_deg_points').value)
        wheel_points = tuple(float(value) for value in
                             self.get_parameter(
                                 'steering_wheel_deg_points').value)
        try:
            return SteeringCalibration(
                command_points=command_points,
                servo_deg_points=servo_points,
                wheel_deg_points=wheel_points,
                wheelbase_m=float(self.get_parameter('wheelbase_m').value),
            )
        except ValueError as exc:
            raise ValueError(f'invalid steering calibration: {exc}') from exc

    def warning_limited(self, message):
        now = time.monotonic()
        if now - self.last_warning_time >= 1.0:
            self.get_logger().warning(message)
            self.last_warning_time = now

    def publish_laser_transform(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.base_frame
        transform.child_frame_id = self.laser_frame
        transform.transform.translation.x = float(
            self.get_parameter('laser_x').value)
        transform.transform.translation.y = float(
            self.get_parameter('laser_y').value)
        transform.transform.translation.z = 0.0
        qx, qy, qz, qw = self.yaw_quaternion(
            float(self.get_parameter('laser_yaw').value))
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.static_tf_broadcaster.sendTransform(transform)

    def receive_loop(self):
        while self.running and rclpy.ok():
            try:
                self.get_logger().info(
                    f'connecting to odometry stream {self.host}:{self.port}')
                with socket.create_connection(
                        (self.host, self.port), timeout=5.0) as sock:
                    sock.settimeout(self.socket_timeout_sec)
                    self.active_socket = sock
                    self.last_sequence = None
                    self.first_packet_after_connect = True
                    self.get_logger().info('odometry stream connected')
                    if self.enable_drive:
                        sock.sendall(b'STOP\n')
                        self.applied_servo_deg = \
                            self.steering_calibration.servo_deg(0.0)
                        self.last_servo_update_time = time.monotonic()
                    with sock.makefile('r', encoding='ascii') as stream:
                        for line in stream:
                            if not self.running:
                                return
                            self.integrate_and_publish(json.loads(line))
                            if self.enable_drive:
                                self.send_drive_command(sock)
            except (
                    OSError, ValueError, TypeError,
                    json.JSONDecodeError) as exc:
                self.warning_limited(f'odometry stream unavailable: {exc}')
                time.sleep(self.reconnect_delay_sec)
            finally:
                self.active_socket = None

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def scan_sector_distance(self, message, center):
        distances = []
        angle = message.angle_min
        for distance in message.ranges:
            delta = self.normalize_angle(angle - center)
            if (abs(delta) <= self.front_sector_half_width_rad and
                    math.isfinite(distance) and
                    message.range_min <= distance <= message.range_max):
                distances.append(float(distance))
            angle += message.angle_increment
        if not distances:
            return math.inf
        distances.sort()
        return distances[min(2, len(distances) - 1)]

    def scan_callback(self, message):
        with self.command_lock:
            self.front_distance_m = self.scan_sector_distance(
                message, self.front_sector_center_rad)
            self.rear_distance_m = self.scan_sector_distance(
                message,
                self.normalize_angle(self.front_sector_center_rad + math.pi))
            self.last_scan_time = time.monotonic()

    def cmd_vel_callback(self, message):
        linear = float(message.linear.x)
        angular = float(message.angular.z)
        if not math.isfinite(linear) or not math.isfinite(angular):
            self.warning_limited('ignored non-finite cmd_vel')
            return
        with self.command_lock:
            self.command_linear = max(
                -self.max_drive_speed_mps,
                min(self.max_drive_speed_mps, linear))
            self.command_angular = max(
                -self.max_steering_command,
                min(self.max_steering_command, angular))
            self.last_command_time = time.monotonic()

    def steering_from_command(self, angular):
        command = max(
            -self.max_steering_command,
            min(self.max_steering_command, angular))
        return self.steering_calibration.servo_deg(command)

    def rate_limited_steering(self, angular, now):
        target = self.steering_from_command(angular)
        dt = 0.01
        if self.last_servo_update_time > 0.0:
            dt = max(0.001, min(0.2, now - self.last_servo_update_time))
        maximum_delta = self.max_servo_rate_deg_per_sec * dt
        delta = max(
            -maximum_delta,
            min(maximum_delta, target - self.applied_servo_deg))
        self.applied_servo_deg += delta
        self.last_servo_update_time = now
        return self.applied_servo_deg

    def send_drive_command(self, sock):
        now = time.monotonic()
        with self.command_lock:
            linear = self.command_linear
            angular = self.command_angular
            command_age = now - self.last_command_time
            scan_age = now - self.last_scan_time
            front_distance = self.front_distance_m
            rear_distance = self.rear_distance_m

        requested_linear = linear
        requested_angular = angular

        stop_reason = ''
        if command_age > self.cmd_vel_timeout_sec:
            linear = 0.0
            angular = 0.0
            stop_reason = 'cmd_vel timeout'
        elif (self.enable_drive_obstacle_safety and
              scan_age > self.scan_timeout_sec):
            linear = 0.0
            stop_reason = 'scan timeout'
        elif self.enable_drive_obstacle_safety:
            motion_distance = (
                front_distance if linear >= 0.0 else rear_distance)
            if not math.isfinite(motion_distance):
                linear = 0.0
                stop_reason = 'no valid obstacle range'
            elif (abs(linear) > 1e-4 and
                  motion_distance <= self.obstacle_stop_distance_m):
                linear = 0.0
                stop_reason = f'obstacle {motion_distance:.2f}m'
            elif (abs(linear) > 1e-4 and
                  motion_distance < self.obstacle_slow_distance_m):
                scale = max(
                    0.25,
                    min(
                        1.0,
                        (motion_distance - self.obstacle_stop_distance_m) /
                        (self.obstacle_slow_distance_m -
                         self.obstacle_stop_distance_m)))
                linear *= scale

        if stop_reason != self.drive_stop_reason:
            if stop_reason:
                self.get_logger().warning(
                    f'remote drive stopped: {stop_reason}')
            elif self.drive_stop_reason:
                self.get_logger().info('remote drive safety condition cleared')
            self.drive_stop_reason = stop_reason

        steering = self.rate_limited_steering(angular, now)
        applied_angular = self.steering_calibration.command_for_servo_deg(
            steering)
        sock.sendall(f'CMD {linear:.4f} {steering:.2f}\n'.encode('ascii'))
        if now - self.last_drive_status_time >= self.drive_status_period_sec:
            wheel_angle = self.steering_calibration.wheel_deg(
                applied_angular)
            turning_radius = self.steering_calibration.turning_radius_m(
                applied_angular)
            status = {
                'requested_speed_mps': requested_linear,
                'applied_speed_mps': linear,
                'requested_steering': requested_angular,
                'applied_steering': applied_angular,
                'servo_deg': steering,
                'front_wheel_deg': wheel_angle,
                'turning_radius_m': turning_radius
                if math.isfinite(turning_radius) else None,
                'stop_reason': stop_reason,
            }
            message = String()
            message.data = json.dumps(
                status, ensure_ascii=True, separators=(',', ':'))
            self.drive_status_publisher.publish(message)
            self.last_drive_status_time = now

    def validate_packet(self, packet):
        if not isinstance(packet, dict):
            raise ValueError('odometry packet is not an object')

        sequence = int(packet.get('seq', -1))
        if sequence < 0:
            raise ValueError('missing odometry sequence')
        if self.last_sequence is not None and sequence <= self.last_sequence:
            raise ValueError(
                f'non-increasing odometry sequence '
                f'{self.last_sequence} -> {sequence}')

        if self.require_imu_ready and packet.get('imu_ready') is not True:
            raise ValueError('IMU is not ready')
        if self.enable_drive and packet.get('remote_drive') is not True:
            raise ValueError(
                'board odometry server is not in mapping-drive mode')

        dt = float(packet.get('dt', 0.0))
        board_mono_ns = int(packet.get('mono_ns', 0))
        if board_mono_ns <= 0:
            raise ValueError('missing board monotonic timestamp')
        gyro_z_dps = (
            float(packet.get('gyro_z_dps', math.nan)) * self.gyro_z_sign)
        left_count = int(packet.get('left', 0))
        right_count = int(packet.get('right', 0))

        if not math.isfinite(dt) or not self.min_dt <= dt <= self.max_dt:
            raise ValueError(
                f'invalid odometry dt={dt:.6f}s, '
                f'expected [{self.min_dt}, {self.max_dt}]')
        if (abs(left_count) > self.max_counts_per_packet or
                abs(right_count) > self.max_counts_per_packet):
            raise ValueError(
                f'encoder spike left={left_count} right={right_count}')
        if not math.isfinite(gyro_z_dps):
            raise ValueError('gyro_z_dps is not finite')

        left_distance = left_count / self.left_counts_per_meter
        right_distance = right_count / self.right_counts_per_meter
        distance = 0.5 * (left_distance + right_distance)
        linear_velocity = distance / dt

        if abs(gyro_z_dps) < self.gyro_deadband_dps:
            gyro_z_dps = 0.0
        angular_velocity = math.radians(gyro_z_dps)
        if abs(linear_velocity) > self.max_linear_speed_mps:
            raise ValueError(
                f'unrealistic linear speed {linear_velocity:.3f} m/s')
        if abs(angular_velocity) > self.max_angular_speed_rps:
            raise ValueError(
                f'unrealistic angular speed {angular_velocity:.3f} rad/s')

        self.last_sequence = sequence
        return (
            dt,
            distance,
            linear_velocity,
            angular_velocity,
            board_mono_ns,
            left_count,
            right_count,
        )

    @staticmethod
    def set_planar_covariance(covariance, xy, yaw):
        covariance[0] = xy
        covariance[7] = xy
        covariance[14] = 99999.0
        covariance[21] = 99999.0
        covariance[28] = 99999.0
        covariance[35] = yaw

    def integrate_and_publish(self, packet):
        if self.first_packet_after_connect:
            self.first_packet_after_connect = False
            if isinstance(packet, dict):
                sequence = int(packet.get('seq', -1))
                if sequence >= 0:
                    self.last_sequence = sequence
            return

        try:
            (
                dt,
                distance,
                linear_velocity,
                angular_velocity,
                board_mono_ns,
                left_count,
                right_count,
            ) = self.validate_packet(packet)
        except (ValueError, TypeError, OverflowError) as exc:
            self.rejected_packets += 1
            self.warning_limited(f'rejected odometry packet: {exc}')
            return

        delta_yaw = angular_velocity * dt
        heading_mid = self.yaw + 0.5 * delta_yaw
        self.x += distance * math.cos(heading_mid)
        self.y += distance * math.sin(heading_mid)
        self.yaw = math.atan2(
            math.sin(self.yaw + delta_yaw), math.cos(self.yaw + delta_yaw))

        receive_time = self.get_clock().now()
        stamp_ns = self.board_clock.map_ns(
            board_mono_ns, receive_time.nanoseconds)
        stamp = Time(nanoseconds=stamp_ns).to_msg()
        qx, qy, qz, qw = self.yaw_quaternion(self.yaw)
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = self.odom_frame
        message.child_frame_id = self.base_frame
        message.pose.pose.position.x = self.x
        message.pose.pose.position.y = self.y
        message.pose.pose.orientation.x = qx
        message.pose.pose.orientation.y = qy
        message.pose.pose.orientation.z = qz
        message.pose.pose.orientation.w = qw
        message.twist.twist.linear.x = linear_velocity
        message.twist.twist.angular.z = angular_velocity
        self.set_planar_covariance(message.pose.covariance, 0.02, 0.03)
        self.set_planar_covariance(message.twist.covariance, 0.04, 0.04)
        self.publisher.publish(message)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(transform)

        self.published_packets += 1
        now = time.monotonic()
        if now - self.last_stats_time >= self.stats_period_sec:
            self.get_logger().info(
                f'odometry healthy: published={self.published_packets} '
                f'rejected={self.rejected_packets} dt={dt:.4f}s '
                f'counts=({left_count},{right_count}) '
                f'v={linear_velocity:.3f}m/s '
                f'w={angular_velocity:.3f}rad/s')
            self.last_stats_time = now

    def destroy_node(self):
        self.running = False
        if self.active_socket is not None:
            try:
                self.active_socket.sendall(b'STOP\n')
            except OSError:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TcpOdometry()
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
