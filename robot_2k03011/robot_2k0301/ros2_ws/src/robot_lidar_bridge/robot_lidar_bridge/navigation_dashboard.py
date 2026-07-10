import json
import math
import mimetypes
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path as FilePath
from urllib.parse import unquote, urlparse

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import (
    PoseStamped,
    PoseWithCovarianceStamped,
)
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Empty, String
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_quaternion(yaw):
    return 0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw)


def finite_float(value, name):
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f'{name} must be finite')
    return result


class DashboardHttpServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address, handler, dashboard):
        super().__init__(address, handler)
        self.dashboard = dashboard


class DashboardRequestHandler(BaseHTTPRequestHandler):
    server_version = 'RobotDashboard/1.0'

    def log_message(self, _format, *_args):
        return

    @property
    def dashboard(self):
        return self.server.dashboard

    def send_json(self, payload, status=HTTPStatus.OK):
        data = json.dumps(
            payload, ensure_ascii=False, separators=(',', ':')).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(data)

    def read_json(self):
        length = int(self.headers.get('Content-Length', '0'))
        if length <= 0 or length > 65536:
            raise ValueError('invalid request body size')
        return json.loads(self.rfile.read(length).decode('utf-8'))

    def serve_static(self, request_path):
        relative = (
            'index.html' if request_path == '/'
            else unquote(request_path).lstrip('/'))
        relative_path = FilePath(relative)
        if (
                relative_path.is_absolute() or
                any(part in ('', '.', '..') for part in relative_path.parts)):
            self.send_error(HTTPStatus.FORBIDDEN)
            return
        candidate = self.dashboard.web_root / relative_path
        if not candidate.is_file():
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        content = candidate.read_bytes()
        content_type = mimetypes.guess_type(candidate.name)[0]
        if content_type is None:
            content_type = 'application/octet-stream'
        self.send_response(HTTPStatus.OK)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(content)))
        self.send_header('Cache-Control', 'no-cache')
        self.end_headers()
        self.wfile.write(content)

    def serve_events(self):
        self.send_response(HTTPStatus.OK)
        self.send_header('Content-Type', 'text/event-stream; charset=utf-8')
        self.send_header('Cache-Control', 'no-cache')
        self.send_header('Connection', 'keep-alive')
        self.end_headers()
        try:
            while self.dashboard.running and rclpy.ok():
                payload = json.dumps(
                    self.dashboard.state_snapshot(),
                    ensure_ascii=False,
                    separators=(',', ':'))
                self.wfile.write(f'data:{payload}\n\n'.encode('utf-8'))
                self.wfile.flush()
                time.sleep(0.25)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass

    def do_GET(self):
        request_path = urlparse(self.path).path
        if request_path == '/api/state':
            self.send_json(self.dashboard.state_snapshot())
        elif request_path == '/api/map':
            self.send_json(self.dashboard.map_snapshot())
        elif request_path == '/api/events':
            self.serve_events()
        elif request_path == '/' or request_path in (
                '/app.js', '/styles.css'):
            self.serve_static(request_path)
        else:
            self.send_error(HTTPStatus.NOT_FOUND)

    def do_POST(self):
        request_path = urlparse(self.path).path
        try:
            payload = self.read_json() if request_path not in (
                '/api/cancel', '/api/emergency-stop',
                '/api/emergency-release', '/api/patrol/cancel') else {}
            if request_path == '/api/goal':
                result = self.dashboard.set_goal(payload)
            elif request_path == '/api/initial-pose':
                result = self.dashboard.set_initial_pose(payload)
            elif request_path == '/api/cancel':
                result = self.dashboard.cancel_navigation()
            elif request_path == '/api/emergency-stop':
                result = self.dashboard.set_emergency_stop(True)
            elif request_path == '/api/emergency-release':
                result = self.dashboard.set_emergency_stop(False)
            elif request_path == '/api/patrol/start':
                result = self.dashboard.start_patrol(payload)
            elif request_path == '/api/patrol/cancel':
                result = self.dashboard.cancel_patrol()
            else:
                self.send_error(HTTPStatus.NOT_FOUND)
                return
            self.send_json({'ok': True, **result})
        except (ValueError, TypeError, KeyError, json.JSONDecodeError) as exc:
            self.send_json(
                {'ok': False, 'error': str(exc)},
                status=HTTPStatus.BAD_REQUEST)


class NavigationDashboard(Node):
    def __init__(self):
        super().__init__('navigation_dashboard')
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8080)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('scan_stride', 3)
        self.declare_parameter('patrol_pause_sec', 1.0)

        self.host = str(self.get_parameter('host').value)
        self.port = int(self.get_parameter('port').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.scan_stride = max(
            int(self.get_parameter('scan_stride').value), 1)
        self.patrol_pause_sec = max(
            float(self.get_parameter('patrol_pause_sec').value), 0.0)
        self.web_root = (
            FilePath(get_package_share_directory('robot_lidar_bridge')) / 'web'
        ).resolve()

        transient_qos = QoSProfile(depth=1)
        transient_qos.reliability = ReliabilityPolicy.RELIABLE
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.goal_publisher = self.create_publisher(
            PoseStamped, '/goal_pose', 10)
        self.initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.cancel_publisher = self.create_publisher(
            Empty, '/navigation/cancel', 10)
        self.emergency_publisher = self.create_publisher(
            Bool, '/navigation/emergency_stop', 10)
        self.map_subscription = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, transient_qos)
        self.scan_subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.path_subscription = self.create_subscription(
            Path, '/planned_path', self.path_callback, transient_qos)
        self.planner_status_subscription = self.create_subscription(
            String, '/planner/status', self.planner_status_callback,
            transient_qos)
        self.navigation_status_subscription = self.create_subscription(
            String, '/navigation/status', self.navigation_status_callback,
            transient_qos)
        self.localization_status_subscription = self.create_subscription(
            String,
            '/navigation/localization_status',
            self.localization_status_callback,
            transient_qos)
        self.obstacle_status_subscription = self.create_subscription(
            String,
            '/navigation/obstacle_status',
            self.obstacle_status_callback,
            transient_qos)
        self.drive_status_subscription = self.create_subscription(
            String, '/drive/status', self.drive_status_callback, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.update_runtime_state)

        self.state_lock = threading.Lock()
        self.started_at = time.time()
        self.event_sequence = 0
        self.events = []
        self.map_data = None
        self.map_version = 0
        self.robot_pose = None
        self.robot_pose_time = 0.0
        self.scan_points = []
        self.scan_time = 0.0
        self.path_points = []
        self.active_goal = None
        self.planner_status = {'state': 'starting'}
        self.navigation_status = {'state': 'starting'}
        self.localization_status = {
            'state': 'initializing', 'ok': False, 'quality': 0.0}
        self.obstacle_status = {
            'state': 'clear', 'blocked': False, 'dynamic_points': 0}
        self.drive_status = {}
        self.emergency_stop = False
        self.patrol = {
            'active': False,
            'repeat': False,
            'index': -1,
            'waypoints': [],
            'state': 'idle',
        }
        self.next_patrol_goal_due = 0.0
        self.running = True
        self.http_server = DashboardHttpServer(
            (self.host, self.port), DashboardRequestHandler, self)
        self.http_thread = threading.Thread(
            target=self.http_server.serve_forever, daemon=True)
        self.http_thread.start()
        self.record_event('info', '上位机服务已启动', source='dashboard')
        self.get_logger().info(
            f'navigation dashboard ready: http://127.0.0.1:{self.port}')

    @staticmethod
    def decode_status(message):
        try:
            value = json.loads(message.data)
            return value if isinstance(value, dict) else {'state': str(value)}
        except json.JSONDecodeError:
            return {'state': message.data}

    def append_event_locked(self, level, text, source='system'):
        self.event_sequence += 1
        self.events.insert(0, {
            'id': self.event_sequence,
            'time': time.time(),
            'level': level,
            'source': source,
            'text': text,
        })
        del self.events[80:]

    def record_event(self, level, text, source='system'):
        with self.state_lock:
            self.append_event_locked(level, text, source)

    def map_callback(self, message):
        origin = message.info.origin
        payload = {
            'frame': message.header.frame_id or self.map_frame,
            'width': int(message.info.width),
            'height': int(message.info.height),
            'resolution': float(message.info.resolution),
            'origin': {
                'x': float(origin.position.x),
                'y': float(origin.position.y),
                'yaw': yaw_from_quaternion(origin.orientation),
            },
            'data': [int(value) for value in message.data],
        }
        with self.state_lock:
            first_map = self.map_data is None
            self.map_version += 1
            payload['version'] = self.map_version
            self.map_data = payload
            if first_map:
                self.append_event_locked(
                    'ok',
                    f'地图已加载 {payload["width"]}x{payload["height"]}',
                    source='map')

    def scan_callback(self, message):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                message.header.frame_id,
                rclpy.time.Time())
        except TransformException:
            return
        translation = transform.transform.translation
        transform_yaw = yaw_from_quaternion(transform.transform.rotation)
        points = []
        angle = message.angle_min
        for index, distance in enumerate(message.ranges):
            if (index % self.scan_stride == 0 and
                    math.isfinite(distance) and
                    message.range_min <= distance <= message.range_max):
                world_angle = transform_yaw + angle
                points.append([
                    translation.x + distance * math.cos(world_angle),
                    translation.y + distance * math.sin(world_angle),
                ])
            angle += message.angle_increment
        with self.state_lock:
            self.scan_points = points
            self.scan_time = time.monotonic()

    def path_callback(self, message):
        points = [
            [pose.pose.position.x, pose.pose.position.y]
            for pose in message.poses
        ]
        with self.state_lock:
            self.path_points = points

    def planner_status_callback(self, message):
        status = self.decode_status(message)
        with self.state_lock:
            previous_state = self.planner_status.get('state')
            self.planner_status = status
            state = status.get('state')
            if state != previous_state:
                level = 'error' if state == 'failed' else 'info'
                self.append_event_locked(
                    level, f'规划器状态：{state}', source='planner')

    def localization_status_callback(self, message):
        status = self.decode_status(message)
        with self.state_lock:
            previous_state = self.localization_status.get('state')
            self.localization_status = status
            state = status.get('state')
            if state != previous_state:
                level = 'ok' if status.get('ok') else 'warn'
                reason = status.get('reason')
                detail = f'：{reason}' if reason else ''
                self.append_event_locked(
                    level,
                    f'定位质量：{state}{detail}',
                    source='localization')

    def obstacle_status_callback(self, message):
        status = self.decode_status(message)
        with self.state_lock:
            was_blocked = bool(self.obstacle_status.get('blocked'))
            self.obstacle_status = status
            blocked = bool(status.get('blocked'))
            if blocked != was_blocked:
                level = 'warn' if blocked else 'ok'
                text = '动态障碍占用当前路径' if blocked else '当前路径障碍已清除'
                self.append_event_locked(level, text, source='obstacle')

    def drive_status_callback(self, message):
        status = self.decode_status(message)
        with self.state_lock:
            self.drive_status = status

    def navigation_status_callback(self, message):
        status = self.decode_status(message)
        with self.state_lock:
            previous_state = self.navigation_status.get('state')
            self.navigation_status = status
            state = status.get('state')
            if state != previous_state:
                level = 'error' if state in (
                    'failed', 'emergency_stopped') else 'info'
                self.append_event_locked(
                    level, f'导航状态：{state}', source='navigation')
            if not self.patrol['active']:
                return
            if state == 'reached':
                last_index = len(self.patrol['waypoints']) - 1
                if self.patrol['index'] >= last_index:
                    if self.patrol['repeat'] and last_index >= 0:
                        self.patrol['index'] = -1
                        self.patrol['state'] = 'waiting'
                        self.next_patrol_goal_due = (
                            time.monotonic() + self.patrol_pause_sec)
                    else:
                        self.patrol['active'] = False
                        self.patrol['state'] = 'completed'
                else:
                    self.patrol['state'] = 'waiting'
                    self.next_patrol_goal_due = (
                        time.monotonic() + self.patrol_pause_sec)
            elif state in ('failed', 'cancelled', 'emergency_stopped'):
                self.patrol['active'] = False
                self.patrol['state'] = state

    def update_runtime_state(self):
        now = time.monotonic()
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
            translation = transform.transform.translation
            pose = {
                'x': float(translation.x),
                'y': float(translation.y),
                'yaw': yaw_from_quaternion(transform.transform.rotation),
            }
            with self.state_lock:
                self.robot_pose = pose
                self.robot_pose_time = now
        except TransformException:
            pass

        waypoint = None
        with self.state_lock:
            if (self.patrol['active'] and
                    self.patrol['state'] == 'waiting' and
                    now >= self.next_patrol_goal_due):
                self.patrol['index'] += 1
                index = self.patrol['index']
                if 0 <= index < len(self.patrol['waypoints']):
                    waypoint = dict(self.patrol['waypoints'][index])
                    self.patrol['state'] = 'navigating'
        if waypoint is not None:
            self.publish_goal(waypoint)

    def state_snapshot(self):
        now = time.monotonic()
        with self.state_lock:
            return {
                'robot': self.robot_pose,
                'robot_connected': (
                    self.robot_pose is not None and
                    now - self.robot_pose_time < 1.0),
                'scan': list(self.scan_points),
                'lidar_connected': (
                    bool(self.scan_points) and
                    now - self.scan_time < 1.0),
                'path': list(self.path_points),
                'goal': dict(self.active_goal)
                if self.active_goal is not None else None,
                'planner': dict(self.planner_status),
                'navigation': dict(self.navigation_status),
                'localization': dict(self.localization_status),
                'obstacle': dict(self.obstacle_status),
                'drive': dict(self.drive_status),
                'emergency_stop': self.emergency_stop,
                'patrol': {
                    'active': self.patrol['active'],
                    'repeat': self.patrol['repeat'],
                    'index': self.patrol['index'],
                    'state': self.patrol['state'],
                    'waypoints': [
                        dict(item) for item in self.patrol['waypoints']
                    ],
                },
                'events': [dict(item) for item in self.events],
                'system': {
                    'uptime_sec': max(0.0, time.time() - self.started_at),
                    'map_frame': self.map_frame,
                    'base_frame': self.base_frame,
                    'dashboard_port': self.port,
                },
                'map_version': self.map_version,
            }

    def map_snapshot(self):
        with self.state_lock:
            if self.map_data is None:
                return {'ready': False, 'version': self.map_version}
            return {'ready': True, **self.map_data}

    @staticmethod
    def parse_pose(payload):
        return {
            'x': finite_float(payload['x'], 'x'),
            'y': finite_float(payload['y'], 'y'),
            'yaw': finite_float(payload.get('yaw', 0.0), 'yaw'),
            'name': str(payload.get('name', '目标点'))[:40],
        }

    def publish_goal(self, pose):
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.map_frame
        message.pose.position.x = pose['x']
        message.pose.position.y = pose['y']
        qx, qy, qz, qw = yaw_quaternion(pose['yaw'])
        message.pose.orientation.x = qx
        message.pose.orientation.y = qy
        message.pose.orientation.z = qz
        message.pose.orientation.w = qw
        with self.state_lock:
            self.active_goal = dict(pose)
        self.goal_publisher.publish(message)
        self.record_event(
            'ok',
            f'目标已下发 {pose["x"]:.2f}, {pose["y"]:.2f}',
            source='mission')

    def set_goal(self, payload):
        pose = self.parse_pose(payload)
        with self.state_lock:
            self.patrol['active'] = False
            self.patrol['state'] = 'idle'
        self.publish_goal(pose)
        return {'goal': pose}

    def set_initial_pose(self, payload):
        pose = self.parse_pose(payload)
        message = PoseWithCovarianceStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.map_frame
        message.pose.pose.position.x = pose['x']
        message.pose.pose.position.y = pose['y']
        qx, qy, qz, qw = yaw_quaternion(pose['yaw'])
        message.pose.pose.orientation.x = qx
        message.pose.pose.orientation.y = qy
        message.pose.pose.orientation.z = qz
        message.pose.pose.orientation.w = qw
        message.pose.covariance[0] = 0.04
        message.pose.covariance[7] = 0.04
        message.pose.covariance[35] = 0.09
        self.initial_pose_publisher.publish(message)
        self.record_event(
            'ok',
            f'初始位姿已发布 {pose["x"]:.2f}, {pose["y"]:.2f}',
            source='localization')
        return {'initial_pose': pose}

    def cancel_navigation(self):
        with self.state_lock:
            self.patrol['active'] = False
            self.patrol['state'] = 'cancelled'
            self.active_goal = None
        self.cancel_publisher.publish(Empty())
        self.record_event('warn', '导航任务已取消', source='mission')
        return {'state': 'cancelled'}

    def set_emergency_stop(self, enabled):
        with self.state_lock:
            self.emergency_stop = bool(enabled)
            if enabled:
                self.patrol['active'] = False
                self.patrol['state'] = 'emergency_stopped'
                self.active_goal = None
        message = Bool()
        message.data = bool(enabled)
        self.emergency_publisher.publish(message)
        if enabled:
            self.cancel_publisher.publish(Empty())
            self.record_event('error', '急停已触发', source='safety')
        else:
            self.record_event('ok', '急停已解除', source='safety')
        return {'emergency_stop': bool(enabled)}

    def start_patrol(self, payload):
        raw_waypoints = payload.get('waypoints')
        if not isinstance(raw_waypoints, list) or not raw_waypoints:
            raise ValueError('waypoints must be a non-empty list')
        if len(raw_waypoints) > 100:
            raise ValueError('waypoints exceeds the limit of 100')
        waypoints = [self.parse_pose(item) for item in raw_waypoints]
        repeat = bool(payload.get('repeat', False))
        with self.state_lock:
            if self.emergency_stop:
                raise ValueError('release emergency stop before patrol')
            self.patrol = {
                'active': True,
                'repeat': repeat,
                'index': 0,
                'waypoints': waypoints,
                'state': 'navigating',
            }
        self.publish_goal(waypoints[0])
        self.record_event(
            'ok',
            f'巡检任务已启动，共 {len(waypoints)} 个点',
            source='patrol')
        return {
            'patrol': {
                'active': True,
                'repeat': repeat,
                'waypoint_count': len(waypoints),
            }
        }

    def cancel_patrol(self):
        return self.cancel_navigation()

    def destroy_node(self):
        self.running = False
        self.http_server.shutdown()
        self.http_server.server_close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NavigationDashboard()
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
