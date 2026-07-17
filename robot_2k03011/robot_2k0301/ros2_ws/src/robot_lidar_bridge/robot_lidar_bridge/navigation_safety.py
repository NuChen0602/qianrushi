import copy
import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from robot_lidar_bridge.navigation_safety_logic import (
    GridSpec,
    LocalizationThresholds,
    StaticGrid,
    evaluate_localization,
    path_blockage,
)


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class NavigationSafety(Node):
    """Builds a temporary obstacle map and gates navigation on localization."""

    def __init__(self):
        super().__init__('navigation_safety')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter(
            'planning_map_topic', '/navigation/planning_map')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('path_topic', '/planned_path')
        self.declare_parameter('amcl_pose_topic', '/amcl_pose')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter(
            'localization_status_topic', '/navigation/localization_status')
        self.declare_parameter(
            'localization_ok_topic', '/navigation/localization_ok')
        self.declare_parameter(
            'obstacle_status_topic', '/navigation/obstacle_status')
        self.declare_parameter(
            'path_blocked_topic', '/navigation/path_blocked')
        self.declare_parameter('enable_dynamic_obstacle_layer', True)
        self.declare_parameter('publish_rate_hz', 5.0)
        self.declare_parameter('scan_stride', 2)
        self.declare_parameter('dynamic_obstacle_ttl_sec', 2.0)
        self.declare_parameter('dynamic_inflation_radius_m', 0.11)
        self.declare_parameter('static_match_radius_m', 0.08)
        self.declare_parameter('path_corridor_radius_m', 0.12)
        self.declare_parameter('path_lookahead_m', 0.80)
        self.declare_parameter('min_dynamic_range_m', 0.12)
        self.declare_parameter('max_dynamic_range_m', 2.5)
        self.declare_parameter('occupied_threshold', 65)
        self.declare_parameter('clear_robot_unknown_footprint', True)
        self.declare_parameter('vehicle_length_m', 0.26)
        self.declare_parameter('vehicle_width_m', 0.135)
        self.declare_parameter('footprint_clearance_margin_m', 0.02)
        self.declare_parameter('block_confirm_samples', 2)
        self.declare_parameter('clear_confirm_samples', 2)
        self.declare_parameter('localization_good_samples', 3)
        self.declare_parameter('pose_timeout_sec', 2.0)
        self.declare_parameter('scan_timeout_sec', 0.8)
        self.declare_parameter('odom_timeout_sec', 0.8)
        self.declare_parameter('tf_localization_fallback', True)
        self.declare_parameter('tf_xy_std_m', 0.03)
        self.declare_parameter('tf_yaw_std_deg', 3.0)
        self.declare_parameter('warn_xy_std_m', 0.12)
        self.declare_parameter('fail_xy_std_m', 0.25)
        self.declare_parameter('warn_yaw_std_deg', 15.0)
        self.declare_parameter('fail_yaw_std_deg', 30.0)

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.enable_dynamic_obstacle_layer = bool(
            self.get_parameter('enable_dynamic_obstacle_layer').value)
        self.scan_stride = max(
            int(self.get_parameter('scan_stride').value), 1)
        self.dynamic_obstacle_ttl_sec = max(
            float(self.get_parameter('dynamic_obstacle_ttl_sec').value), 0.2)
        self.dynamic_inflation_radius_m = max(
            float(self.get_parameter('dynamic_inflation_radius_m').value), 0.0)
        self.static_match_radius_m = max(
            float(self.get_parameter('static_match_radius_m').value), 0.0)
        self.path_corridor_radius_m = max(
            float(self.get_parameter('path_corridor_radius_m').value), 0.02)
        self.path_lookahead_m = max(
            float(self.get_parameter('path_lookahead_m').value), 0.1)
        self.min_dynamic_range_m = max(
            float(self.get_parameter('min_dynamic_range_m').value), 0.02)
        self.max_dynamic_range_m = max(
            float(self.get_parameter('max_dynamic_range_m').value),
            self.min_dynamic_range_m)
        self.occupied_threshold = int(
            self.get_parameter('occupied_threshold').value)
        self.clear_robot_unknown_footprint = bool(
            self.get_parameter('clear_robot_unknown_footprint').value)
        self.vehicle_length_m = max(
            float(self.get_parameter('vehicle_length_m').value), 0.01)
        self.vehicle_width_m = max(
            float(self.get_parameter('vehicle_width_m').value), 0.01)
        self.footprint_clearance_margin_m = max(
            float(self.get_parameter('footprint_clearance_margin_m').value), 0.0)
        self.block_confirm_samples = max(
            int(self.get_parameter('block_confirm_samples').value), 1)
        self.clear_confirm_samples = max(
            int(self.get_parameter('clear_confirm_samples').value), 1)
        self.localization_good_samples = max(
            int(self.get_parameter('localization_good_samples').value), 1)
        self.localization_thresholds = LocalizationThresholds(
            pose_timeout_sec=max(
                float(self.get_parameter('pose_timeout_sec').value), 0.1),
            scan_timeout_sec=max(
                float(self.get_parameter('scan_timeout_sec').value), 0.1),
            odom_timeout_sec=max(
                float(self.get_parameter('odom_timeout_sec').value), 0.1),
            warn_xy_std_m=max(
                float(self.get_parameter('warn_xy_std_m').value), 0.01),
            fail_xy_std_m=max(
                float(self.get_parameter('fail_xy_std_m').value), 0.02),
            warn_yaw_std_rad=math.radians(max(
                float(self.get_parameter('warn_yaw_std_deg').value), 1.0)),
            fail_yaw_std_rad=math.radians(max(
                float(self.get_parameter('fail_yaw_std_deg').value), 2.0)),
        )
        self.tf_localization_fallback = bool(
            self.get_parameter('tf_localization_fallback').value)
        self.tf_xy_std_m = max(
            float(self.get_parameter('tf_xy_std_m').value), 0.001)
        self.tf_yaw_std_rad = math.radians(max(
            float(self.get_parameter('tf_yaw_std_deg').value), 0.1))

        transient_qos = QoSProfile(depth=1)
        transient_qos.reliability = ReliabilityPolicy.RELIABLE
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.planning_map_pub = self.create_publisher(
            OccupancyGrid,
            str(self.get_parameter('planning_map_topic').value),
            transient_qos)
        self.localization_status_pub = self.create_publisher(
            String,
            str(self.get_parameter('localization_status_topic').value),
            transient_qos)
        self.localization_ok_pub = self.create_publisher(
            Bool,
            str(self.get_parameter('localization_ok_topic').value),
            transient_qos)
        self.obstacle_status_pub = self.create_publisher(
            String,
            str(self.get_parameter('obstacle_status_topic').value),
            transient_qos)
        self.path_blocked_pub = self.create_publisher(
            Bool,
            str(self.get_parameter('path_blocked_topic').value),
            transient_qos)

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter('map_topic').value),
            self.map_callback,
            transient_qos)
        self.scan_sub = self.create_subscription(
            LaserScan,
            str(self.get_parameter('scan_topic').value),
            self.scan_callback,
            10)
        self.path_sub = self.create_subscription(
            Path,
            str(self.get_parameter('path_topic').value),
            self.path_callback,
            transient_qos)
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter('amcl_pose_topic').value),
            self.amcl_callback,
            10)
        self.odom_sub = self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self.odom_callback,
            20)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        rate = max(float(self.get_parameter('publish_rate_hz').value), 1.0)
        self.timer = self.create_timer(1.0 / rate, self.update)

        self.base_map_message = None
        self.static_grid = None
        self.dynamic_cells = {}
        self.path_points = []
        self.amcl_covariance = None
        self.last_pose_time = -math.inf
        self.last_scan_time = -math.inf
        self.last_odom_time = -math.inf
        self.localization_ok = False
        self.localization_good_count = 0
        self.last_localization_state = ''
        self.path_blocked = False
        self.blocked_count = 0
        self.clear_count = 0
        self.last_obstacle_state = ''

        dynamic_layer = (
            'enabled' if self.enable_dynamic_obstacle_layer else 'disabled')
        self.get_logger().info(
            f'navigation safety ready: dynamic obstacle layer {dynamic_layer}; '
            'localization quality gate enabled')

    def map_callback(self, message):
        origin = message.info.origin
        spec = GridSpec(
            width=int(message.info.width),
            height=int(message.info.height),
            resolution=float(message.info.resolution),
            origin_x=float(origin.position.x),
            origin_y=float(origin.position.y),
            origin_yaw=yaw_from_quaternion(origin.orientation),
        )
        self.static_grid = StaticGrid(
            spec, message.data, self.occupied_threshold)
        self.base_map_message = copy.deepcopy(message)
        self.dynamic_cells.clear()
        self.publish_planning_map(time.monotonic())
        self.get_logger().info(
            f'safety map received: {spec.width}x{spec.height} '
            f'resolution={spec.resolution:.3f}m')

    def scan_callback(self, message):
        now = time.monotonic()
        self.last_scan_time = now
        if (not self.enable_dynamic_obstacle_layer or
                self.static_grid is None):
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                message.header.frame_id,
                rclpy.time.Time())
        except TransformException:
            return

        translation = transform.transform.translation
        transform_yaw = yaw_from_quaternion(transform.transform.rotation)
        angle = message.angle_min
        expiry = now + self.dynamic_obstacle_ttl_sec
        for index, distance in enumerate(message.ranges):
            if (index % self.scan_stride == 0 and
                    math.isfinite(distance) and
                    message.range_min <= distance <= message.range_max and
                    self.min_dynamic_range_m <= distance <=
                    self.max_dynamic_range_m):
                world_angle = transform_yaw + angle
                x = translation.x + distance * math.cos(world_angle)
                y = translation.y + distance * math.sin(world_angle)
                if not self.static_grid.occupied_near_world(
                        x, y, self.static_match_radius_m):
                    cell = self.static_grid.spec.world_to_grid(x, y)
                    if self.static_grid.spec.contains(*cell):
                        self.dynamic_cells[cell] = expiry
            angle += message.angle_increment

    def path_callback(self, message):
        if not self.enable_dynamic_obstacle_layer:
            self.path_points = []
            return
        self.path_points = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in message.poses
        ]

    def amcl_callback(self, message):
        self.amcl_covariance = tuple(message.pose.covariance)
        self.last_pose_time = time.monotonic()

    def odom_callback(self, _message):
        self.last_odom_time = time.monotonic()

    def active_dynamic_points(self, now):
        if not self.enable_dynamic_obstacle_layer:
            self.dynamic_cells.clear()
            return []
        expired = [
            cell for cell, expiry in self.dynamic_cells.items()
            if expiry <= now
        ]
        for cell in expired:
            del self.dynamic_cells[cell]
        if self.static_grid is None:
            return []
        return [
            self.static_grid.spec.grid_to_world(*cell)
            for cell in self.dynamic_cells
        ]

    @staticmethod
    def publish_json(publisher, payload):
        message = String()
        message.data = json.dumps(
            payload, ensure_ascii=True, separators=(',', ':'))
        publisher.publish(message)

    def publish_planning_map(self, now):
        if self.base_map_message is None or self.static_grid is None:
            return
        active_cells = [
            cell for cell, expiry in self.dynamic_cells.items()
            if expiry > now
        ]
        output = copy.deepcopy(self.base_map_message)
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = self.map_frame
        data = self.static_grid.inflated_data(
            active_cells, self.dynamic_inflation_radius_m)
        if self.clear_robot_unknown_footprint:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame, self.base_frame, rclpy.time.Time())
                translation = transform.transform.translation
                yaw = yaw_from_quaternion(transform.transform.rotation)
                self.clear_unknown_robot_footprint(
                    data, translation.x, translation.y, yaw)
            except TransformException:
                pass
        output.data = data
        self.planning_map_pub.publish(output)

    def clear_unknown_robot_footprint(self, data, robot_x, robot_y, yaw):
        """Clear only unknown cells physically occupied by the robot itself."""
        spec = self.static_grid.spec
        half_length = self.vehicle_length_m * 0.5 + self.footprint_clearance_margin_m
        half_width = self.vehicle_width_m * 0.5 + self.footprint_clearance_margin_m
        radius = math.hypot(half_length, half_width)
        center_x, center_y = spec.world_to_grid(robot_x, robot_y)
        cells = math.ceil(radius / spec.resolution) + 1
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        for gy in range(center_y - cells, center_y + cells + 1):
            for gx in range(center_x - cells, center_x + cells + 1):
                if not spec.contains(gx, gy):
                    continue
                index = gy * spec.width + gx
                if data[index] != -1:
                    continue
                wx, wy = spec.grid_to_world(gx, gy)
                dx = wx - robot_x
                dy = wy - robot_y
                local_x = cosine * dx + sine * dy
                local_y = -sine * dx + cosine * dy
                if abs(local_x) <= half_length and abs(local_y) <= half_width:
                    data[index] = 0

    def update_localization(self, now):
        covariance = self.amcl_covariance or ()
        pose_age = now - self.last_pose_time
        pose_source = 'amcl'
        if (self.tf_localization_fallback and
                (not math.isfinite(pose_age) or
                 pose_age > self.localization_thresholds.pose_timeout_sec)):
            try:
                self.tf_buffer.lookup_transform(
                    self.map_frame, self.base_frame, rclpy.time.Time())
                pose_age = 0.0
                pose_source = 'tf'
                covariance = [0.0] * 36
                covariance[0] = self.tf_xy_std_m ** 2
                covariance[7] = self.tf_xy_std_m ** 2
                covariance[35] = self.tf_yaw_std_rad ** 2
            except TransformException:
                pass
        raw = evaluate_localization(
            covariance,
            pose_age,
            now - self.last_scan_time,
            now - self.last_odom_time,
            self.localization_thresholds)
        if raw['ok']:
            self.localization_good_count += 1
            if self.localization_good_count >= self.localization_good_samples:
                self.localization_ok = True
        else:
            self.localization_good_count = 0
            self.localization_ok = False

        payload = dict(raw)
        payload['ok'] = self.localization_ok
        payload['source'] = pose_source
        if raw['ok'] and not self.localization_ok:
            payload['state'] = 'initializing'
            payload['reason'] = 'quality_confirming'
        bool_message = Bool()
        bool_message.data = self.localization_ok
        self.localization_ok_pub.publish(bool_message)
        self.publish_json(self.localization_status_pub, payload)
        state_key = f'{payload["state"]}:{payload.get("reason", "")}'
        if state_key != self.last_localization_state:
            message = (
                f'localization {payload["state"]}: '
                f'{payload.get("reason") or "quality accepted"} '
                f'quality={payload.get("quality", 0.0):.1f}')
            if self.localization_ok:
                self.get_logger().info(message)
            else:
                self.get_logger().warning(message)
            self.last_localization_state = state_key

    def update_obstacles(self, now, dynamic_points):
        if not self.enable_dynamic_obstacle_layer:
            self.path_blocked = False
            self.blocked_count = 0
            self.clear_count = 0
            bool_message = Bool()
            bool_message.data = False
            self.path_blocked_pub.publish(bool_message)
            payload = {
                'state': 'disabled',
                'blocked': False,
                'dynamic_points': 0,
                'blocking_points': 0,
                'nearest_path_m': None,
                'ttl_sec': 0.0,
            }
            self.publish_json(self.obstacle_status_pub, payload)
            if payload['state'] != self.last_obstacle_state:
                self.get_logger().info(
                    'dynamic lidar obstacle avoidance disabled')
                self.last_obstacle_state = payload['state']
            return

        blocked = False
        nearest_path = math.inf
        blocking_points = 0
        if self.path_points:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame, self.base_frame, rclpy.time.Time())
                blocked, nearest_path, blocking_points = path_blockage(
                    dynamic_points,
                    self.path_points,
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    self.path_lookahead_m,
                    self.path_corridor_radius_m)
            except TransformException:
                blocked = True

        if blocked:
            self.blocked_count += 1
            self.clear_count = 0
            if self.blocked_count >= self.block_confirm_samples:
                self.path_blocked = True
        else:
            self.clear_count += 1
            self.blocked_count = 0
            if self.clear_count >= self.clear_confirm_samples:
                self.path_blocked = False

        bool_message = Bool()
        bool_message.data = self.path_blocked
        self.path_blocked_pub.publish(bool_message)
        payload = {
            'state': 'blocked' if self.path_blocked else 'clear',
            'blocked': self.path_blocked,
            'dynamic_points': len(dynamic_points),
            'blocking_points': blocking_points,
            'nearest_path_m': nearest_path if math.isfinite(nearest_path)
            else None,
            'ttl_sec': self.dynamic_obstacle_ttl_sec,
        }
        self.publish_json(self.obstacle_status_pub, payload)
        if payload['state'] != self.last_obstacle_state:
            message = (
                f'path obstacle state={payload["state"]} '
                f'dynamic_points={len(dynamic_points)} '
                f'blocking_points={blocking_points}')
            if self.path_blocked:
                self.get_logger().warning(message)
            else:
                self.get_logger().info(message)
            self.last_obstacle_state = payload['state']

    def update(self):
        now = time.monotonic()
        dynamic_points = self.active_dynamic_points(now)
        self.publish_planning_map(now)
        self.update_localization(now)
        self.update_obstacles(now, dynamic_points)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationSafety()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
