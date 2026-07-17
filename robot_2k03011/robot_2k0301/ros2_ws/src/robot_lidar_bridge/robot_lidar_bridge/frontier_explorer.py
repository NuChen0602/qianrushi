import json
import math
import time
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, String
from tf2_ros import Buffer, TransformException, TransformListener


def frontier_clusters(data, width, height, min_cells=4):
    """Return connected free-cell groups adjacent to unknown space."""
    frontier = set()
    for y in range(1, height - 1):
        for x in range(1, width - 1):
            index = y * width + x
            if data[index] != 0:
                continue
            if any(data[(y + dy) * width + x + dx] == -1
                   for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1))):
                frontier.add((x, y))
    groups = []
    while frontier:
        seed = frontier.pop()
        group = [seed]
        queue = deque([seed])
        while queue:
            x, y = queue.popleft()
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    neighbor = (x + dx, y + dy)
                    if neighbor in frontier:
                        frontier.remove(neighbor)
                        queue.append(neighbor)
                        group.append(neighbor)
        if len(group) >= min_cells:
            groups.append(group)
    return groups


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        self.declare_parameter('boundary_radius_m', 2.0)
        self.declare_parameter('goal_timeout_sec', 45.0)
        self.declare_parameter('minimum_frontier_cells', 4)
        # Vehicle half-width is 0.0675 m; 0.08 m keeps 1.25 cm side margin
        # while remaining usable in a sparse 2 cm online SLAM grid.
        self.declare_parameter('goal_clearance_m', 0.08)
        self.declare_parameter('preferred_goal_distance_m', 0.45)
        self.declare_parameter('bootstrap_forward_distance_m', 0.70)
        self.declare_parameter('relocation_cooldown_sec', 3.0)
        self.declare_parameter('minimum_turning_radius_m', 0.34)
        self.declare_parameter('allow_reverse_maneuvers', True)
        self.radius = float(self.get_parameter('boundary_radius_m').value)
        self.timeout = float(self.get_parameter('goal_timeout_sec').value)
        self.min_cells = int(self.get_parameter('minimum_frontier_cells').value)
        self.clearance = float(self.get_parameter('goal_clearance_m').value)
        self.preferred_goal_distance = max(
            0.25, float(self.get_parameter('preferred_goal_distance_m').value))
        self.bootstrap_distance = max(
            0.0, float(self.get_parameter('bootstrap_forward_distance_m').value))
        self.relocation_cooldown = max(
            1.0, float(self.get_parameter('relocation_cooldown_sec').value))
        self.minimum_turning_radius = max(
            0.1, float(self.get_parameter('minimum_turning_radius_m').value))
        self.allow_reverse_maneuvers = bool(
            self.get_parameter('allow_reverse_maneuvers').value)
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos)
        self.create_subscription(String, '/navigation/status', self.status_callback, qos)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.cancel_pub = self.create_publisher(Empty, '/navigation/cancel', 10)
        self.state_pub = self.create_publisher(String, '/exploration/status', qos)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.map = None
        self.start = None
        self.goal = None
        self.goal_started = 0.0
        self.nav_state = 'idle'
        self.blacklist = []
        self.no_frontier_count = 0
        self.bootstrap_done = False
        self.last_relocation_time = -math.inf
        self.create_timer(1.0, self.tick)
        self.publish_state('waiting_for_map')

    def publish_state(self, state, **extra):
        payload = {'state': state, 'boundary_radius_m': self.radius, **extra}
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False)
        self.state_pub.publish(message)

    def map_callback(self, message):
        self.map = message

    def status_callback(self, message):
        try:
            self.nav_state = json.loads(message.data).get('state', message.data)
        except (ValueError, AttributeError):
            self.nav_state = message.data

    def robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except TransformException:
            return None
        rotation = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z))
        return transform.transform.translation.x, transform.transform.translation.y, yaw

    @staticmethod
    def angle_difference(a, b):
        return math.atan2(math.sin(a - b), math.cos(a - b))

    def forward_reachable_bearing(self, distance, bearing_error):
        # For a circular Ackermann arc: chord = 2 R sin(bearing). Add a small
        # allowance for multi-primitive Hybrid A* paths and grid discretization.
        ratio = min(1.0, distance / (2.0 * self.minimum_turning_radius))
        limit = math.asin(ratio) + math.radians(10.0)
        return abs(bearing_error) <= min(limit, math.radians(75.0))

    def relocation_goal(self, robot, initial=False):
        if self.bootstrap_distance <= 0.0:
            return None
        info = self.map.info
        cells = self.map.data
        options = []
        stride = max(1, int(round(0.06 / info.resolution)))
        for gy in range(0, info.height, stride):
            for gx in range(0, info.width, stride):
                if cells[gy * info.width + gx] != 0 or not self.safe_cell(gx, gy, cells):
                    continue
                wx = info.origin.position.x + (gx + 0.5) * info.resolution
                wy = info.origin.position.y + (gy + 0.5) * info.resolution
                distance = math.hypot(wx - robot[0], wy - robot[1])
                if distance < 0.25 or distance > self.bootstrap_distance:
                    continue
                if math.hypot(wx - self.start[0], wy - self.start[1]) > self.radius:
                    continue
                heading = math.atan2(wy - robot[1], wx - robot[0])
                heading_error = abs(self.angle_difference(heading, robot[2]))
                # Forward-only Ackermann paths need a target in the forward
                # hemisphere; shallow arcs are substantially more reliable.
                if (not self.allow_reverse_maneuvers and
                        not self.forward_reachable_bearing(distance, heading_error)):
                    continue
                if any(math.hypot(wx - bx, wy - by) < 0.25 for bx, by in self.blacklist):
                    continue
                score = distance - 0.20 * heading_error
                options.append((score, wx, wy, 0, distance))
        if options:
            return max(options)
        return None

    def safe_cell(self, x, y, cells):
        info = self.map.info
        radius = max(1, int(math.ceil(self.clearance / info.resolution)))
        for yy in range(max(0, y - radius), min(info.height, y + radius + 1)):
            for xx in range(max(0, x - radius), min(info.width, x + radius + 1)):
                if cells[yy * info.width + xx] != 0:
                    return False
        return True

    def choose_goal(self, robot):
        info = self.map.info
        cells = self.map.data
        candidates = []
        groups = frontier_clusters(cells, info.width, info.height, self.min_cells)
        diagnostics = {
            'frontier_groups': len(groups),
            'frontier_cells': sum(len(group) for group in groups),
            'within_boundary': 0,
            'safe_goals': 0,
        }
        for group in groups:
            cx = sum(p[0] for p in group) / len(group)
            cy = sum(p[1] for p in group) / len(group)
            # Frontier cells touch unknown space, so they cannot themselves
            # satisfy robot-footprint clearance. Search the explored side for
            # the closest fully-known free cell and use that as the goal.
            search = max(2, int(math.ceil((self.clearance + 0.20) / info.resolution)))
            safe = []
            for fx, fy in group:
                for y in range(max(0, fy - search), min(info.height, fy + search + 1)):
                    for x in range(max(0, fx - search), min(info.width, fx + search + 1)):
                        if cells[y * info.width + x] == 0 and self.safe_cell(x, y, cells):
                            safe.append((x, y))
            if not safe:
                continue
            x, y = min(set(safe), key=lambda p: (p[0] - cx) ** 2 + (p[1] - cy) ** 2)
            wx = info.origin.position.x + (x + 0.5) * info.resolution
            wy = info.origin.position.y + (y + 0.5) * info.resolution
            if math.hypot(wx - self.start[0], wy - self.start[1]) > self.radius:
                continue
            diagnostics['within_boundary'] += 1
            if math.hypot(wx - robot[0], wy - robot[1]) < 0.20:
                continue
            if any(math.hypot(wx - bx, wy - by) < 0.25 for bx, by in self.blacklist):
                continue
            distance = math.hypot(wx - robot[0], wy - robot[1])
            bearing = math.atan2(wy - robot[1], wx - robot[0])
            if (not self.allow_reverse_maneuvers and
                    not self.forward_reachable_bearing(
                        distance, self.angle_difference(bearing, robot[2]))):
                continue
            # Prefer a meaningful viewpoint change. Larger frontier groups
            # promise more new map, while distance helps expose body-occluded
            # areas instead of repeatedly issuing 20 cm micro-goals.
            score = len(group) * 0.08 + distance * 0.80
            candidates.append((score, wx, wy, len(group), distance))
            diagnostics['safe_goals'] += 1
        preferred = [item for item in candidates
                     if item[4] >= self.preferred_goal_distance]
        return max(preferred or candidates, default=None), diagnostics

    def send_goal(self, candidate, robot):
        _, x, y, size, distance = candidate
        message = PoseStamped()
        message.header.frame_id = 'map'
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x = x
        message.pose.position.y = y
        yaw = math.atan2(y - robot[1], x - robot[0])
        message.pose.orientation.z = math.sin(yaw / 2.0)
        message.pose.orientation.w = math.cos(yaw / 2.0)
        self.goal_pub.publish(message)
        self.goal = (x, y)
        self.goal_started = time.monotonic()
        self.publish_state('navigating', goal={'x': x, 'y': y},
                           goal_distance_m=round(distance, 3), frontier_cells=size)

    def tick(self):
        if self.map is None:
            return
        robot = self.robot_pose()
        if robot is None:
            self.publish_state('waiting_for_tf')
            return
        if self.start is None:
            self.start = robot
            self.publish_state('started', start={'x': robot[0], 'y': robot[1]})
        if self.goal is not None:
            timed_out = time.monotonic() - self.goal_started > self.timeout
            if self.nav_state == 'reached':
                self.goal = None
            elif self.nav_state in ('failed', 'cancelled') or timed_out:
                self.blacklist.append(self.goal)
                self.cancel_pub.publish(Empty())
                self.goal = None
            else:
                return
        candidate = None
        if not self.bootstrap_done:
            candidate = self.relocation_goal(robot, initial=True)
        if candidate is not None:
            self.bootstrap_done = True
            self.last_relocation_time = time.monotonic()
            self.send_goal(candidate, robot)
            return
        candidate, diagnostics = self.choose_goal(robot)
        if candidate is None:
            self.no_frontier_count += 1
            if (diagnostics['frontier_groups'] > 0 and
                    time.monotonic() - self.last_relocation_time >= self.relocation_cooldown):
                relocation = self.relocation_goal(robot)
                if relocation is not None:
                    self.last_relocation_time = time.monotonic()
                    self.no_frontier_count = 0
                    self.send_goal(relocation, robot)
                    return
            if self.no_frontier_count >= 5:
                if diagnostics['frontier_groups'] == 0:
                    self.publish_state('completed', message='2m 范围内探索完成', **diagnostics)
                else:
                    self.publish_state('stalled', message='仍有 Frontier，但暂无安全可达目标',
                                       **diagnostics)
            else:
                self.publish_state('checking_completion', remaining_checks=5 - self.no_frontier_count,
                                   **diagnostics)
            return
        self.no_frontier_count = 0
        self.send_goal(candidate, robot)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            try:
                node.cancel_pub.publish(Empty())
            except Exception:
                # Launch may invalidate the ROS context before Python receives
                # SIGINT; the drive bridge independently stops on cmd timeout.
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
