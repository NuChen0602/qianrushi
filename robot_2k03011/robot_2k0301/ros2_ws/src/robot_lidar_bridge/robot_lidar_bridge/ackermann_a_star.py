import heapq
import math
import time
from dataclasses import dataclass


def clamp(value, low, high):
    return max(low, min(high, value))


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class PlanningCancelled(RuntimeError):
    pass


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass
class PlannerConfig:
    xy_resolution: float = 0.04
    yaw_resolution: float = math.radians(10.0)
    primitive_length: float = 0.08
    integration_step: float = 0.02
    wheelbase: float = 0.18
    vehicle_length: float = 0.26
    vehicle_width: float = 0.135
    safety_margin: float = 0.025
    max_steer_angle: float = math.radians(37.0)
    steering_samples: int = 5
    goal_tolerance: float = 0.10
    goal_yaw_tolerance: float = math.radians(10.0)
    relaxed_goal_yaw_tolerance: float = math.radians(45.0)
    start_collision_tolerance: float = 0.08
    approach_goal_on_failure: bool = True
    approach_goal_tolerance: float = 0.35
    allow_goal_yaw_fallback: bool = True
    steer_cost: float = 0.04
    steer_change_cost: float = 0.10
    allow_reverse: bool = True
    reverse_cost_multiplier: float = 1.8
    direction_switch_cost: float = 0.8
    heading_heuristic_weight: float = 0.08
    relaxed_heading_heuristic_weight: float = 0.12
    max_expansions: int = 80000
    planning_timeout_sec: float = 3.0
    planning_stage_timeout_sec: float = 0.0

    def sanitized(self):
        self.xy_resolution = max(self.xy_resolution, 0.01)
        self.yaw_resolution = max(self.yaw_resolution, math.radians(2.0))
        self.primitive_length = max(
            self.primitive_length, self.xy_resolution)
        self.integration_step = clamp(
            self.integration_step, 0.005, self.primitive_length)
        self.wheelbase = max(self.wheelbase, 0.05)
        self.vehicle_length = max(self.vehicle_length, 0.05)
        self.vehicle_width = max(self.vehicle_width, 0.04)
        self.safety_margin = max(self.safety_margin, 0.0)
        self.max_steer_angle = clamp(
            abs(self.max_steer_angle), math.radians(5.0), math.radians(60.0))
        self.steering_samples = max(3, int(self.steering_samples))
        if self.steering_samples % 2 == 0:
            self.steering_samples += 1
        self.goal_tolerance = max(self.goal_tolerance, self.xy_resolution)
        self.start_collision_tolerance = max(
            self.start_collision_tolerance, 0.0)
        self.approach_goal_tolerance = max(
            self.approach_goal_tolerance, self.goal_tolerance)
        self.goal_yaw_tolerance = clamp(
            abs(self.goal_yaw_tolerance),
            self.yaw_resolution,
            math.pi)
        self.relaxed_goal_yaw_tolerance = clamp(
            abs(self.relaxed_goal_yaw_tolerance),
            self.goal_yaw_tolerance,
            math.pi)
        self.reverse_cost_multiplier = max(
            self.reverse_cost_multiplier, 1.0)
        self.direction_switch_cost = max(self.direction_switch_cost, 0.0)
        self.heading_heuristic_weight = max(
            self.heading_heuristic_weight, 0.0)
        self.relaxed_heading_heuristic_weight = max(
            self.relaxed_heading_heuristic_weight, 0.0)
        self.max_expansions = max(int(self.max_expansions), 100)
        self.planning_timeout_sec = float(self.planning_timeout_sec)
        if (not math.isfinite(self.planning_timeout_sec) or
                self.planning_timeout_sec < 0.0):
            self.planning_timeout_sec = 0.0
        self.planning_stage_timeout_sec = float(
            self.planning_stage_timeout_sec)
        if (not math.isfinite(self.planning_stage_timeout_sec) or
                self.planning_stage_timeout_sec < 0.0):
            self.planning_stage_timeout_sec = 0.0
        return self


@dataclass
class PlannedPath:
    poses: list
    cost: float
    expansions: int
    planning_time_sec: float
    relaxed_goal_yaw: bool = False
    approach_goal: bool = False
    requested_goal_distance: float = 0.0
    requested_goal_yaw_error: float = 0.0
    goal_tolerance: float = 0.0
    start_adjusted: bool = False
    start_adjustment_distance: float = 0.0


@dataclass
class _SearchNode:
    pose: Pose2D
    steer: float
    direction: int
    cost: float
    parent_key: tuple
    segment: list


class OccupancyGridMap:
    def __init__(
            self,
            width,
            height,
            resolution,
            origin_x,
            origin_y,
            origin_yaw,
            data,
            occupied_threshold=65,
            unknown_is_occupied=True):
        self.width = int(width)
        self.height = int(height)
        self.resolution = float(resolution)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.origin_yaw = float(origin_yaw)
        self.cos_origin = math.cos(self.origin_yaw)
        self.sin_origin = math.sin(self.origin_yaw)
        self.occupied_threshold = int(occupied_threshold)
        self.unknown_is_occupied = bool(unknown_is_occupied)
        self.data = tuple(int(value) for value in data)
        if self.width <= 0 or self.height <= 0:
            raise ValueError('occupancy grid dimensions must be positive')
        if self.resolution <= 0.0:
            raise ValueError('occupancy grid resolution must be positive')
        if len(self.data) != self.width * self.height:
            raise ValueError('occupancy grid data size does not match dimensions')

    def world_to_grid(self, x, y):
        dx = x - self.origin_x
        dy = y - self.origin_y
        local_x = self.cos_origin * dx + self.sin_origin * dy
        local_y = -self.sin_origin * dx + self.cos_origin * dy
        return (
            math.floor(local_x / self.resolution),
            math.floor(local_y / self.resolution),
        )

    def is_occupied_cell(self, gx, gy):
        if gx < 0 or gy < 0 or gx >= self.width or gy >= self.height:
            return True
        value = self.data[gy * self.width + gx]
        if value < 0:
            return self.unknown_is_occupied
        return value >= self.occupied_threshold

    def is_occupied_world(self, x, y):
        return self.is_occupied_cell(*self.world_to_grid(x, y))


class HybridAStarPlanner:
    """Hybrid A* for a small Ackermann steering robot."""

    def __init__(self, grid_map, config=None):
        self.grid_map = grid_map
        self.config = (config or PlannerConfig()).sanitized()
        self.footprint_samples = self._build_footprint_samples()
        self.steering_inputs = self._build_steering_inputs()

    def _build_steering_inputs(self):
        count = self.config.steering_samples
        maximum = self.config.max_steer_angle
        values = [
            -maximum + (2.0 * maximum * index / (count - 1))
            for index in range(count)
        ]
        values.sort(key=lambda value: (abs(value), value))
        return values

    def _build_footprint_samples(self):
        half_length = 0.5 * self.config.vehicle_length
        half_width = 0.5 * self.config.vehicle_width
        half_length += self.config.safety_margin
        half_width += self.config.safety_margin
        step = max(self.grid_map.resolution * 0.8, 0.015)
        samples = []
        x = -half_length
        while x <= half_length + 1e-9:
            y = -half_width
            while y <= half_width + 1e-9:
                samples.append((x, y))
                y += step
            samples.append((x, half_width))
            x += step
        x = -half_length
        while x <= half_length + 1e-9:
            samples.append((x, half_width))
            samples.append((x, -half_width))
            x += step
        return tuple(samples)

    def pose_is_collision_free(self, pose):
        cos_yaw = math.cos(pose.yaw)
        sin_yaw = math.sin(pose.yaw)
        for local_x, local_y in self.footprint_samples:
            world_x = pose.x + cos_yaw * local_x - sin_yaw * local_y
            world_y = pose.y + sin_yaw * local_x + cos_yaw * local_y
            if self.grid_map.is_occupied_world(world_x, world_y):
                return False
        return True

    def segment_is_collision_free(self, poses):
        return all(self.pose_is_collision_free(pose) for pose in poses)

    def _state_key(self, pose, direction):
        x_index = round((pose.x - self.grid_map.origin_x) /
                        self.config.xy_resolution)
        y_index = round((pose.y - self.grid_map.origin_y) /
                        self.config.xy_resolution)
        yaw_index = round(
            normalize_angle(pose.yaw) / self.config.yaw_resolution)
        return x_index, y_index, yaw_index, int(direction)

    def _heuristic(self, pose, goal, require_goal_yaw):
        distance = math.hypot(goal.x - pose.x, goal.y - pose.y)
        weight = (
            self.config.heading_heuristic_weight
            if require_goal_yaw
            else self.config.relaxed_heading_heuristic_weight)
        if weight <= 0.0:
            return distance
        heading_error = abs(normalize_angle(goal.yaw - pose.yaw))
        return distance + (
            weight * self.config.wheelbase * heading_error)

    def _goal_reached(self, pose, goal, require_goal_yaw, position_tolerance):
        if math.hypot(goal.x - pose.x, goal.y - pose.y) > position_tolerance:
            return False
        yaw_tolerance = (
            self.config.goal_yaw_tolerance
            if require_goal_yaw
            else self.config.relaxed_goal_yaw_tolerance)
        return abs(normalize_angle(goal.yaw - pose.yaw)) <= yaw_tolerance

    def _goal_position_offsets(self, position_tolerance):
        step = max(
            min(self.grid_map.resolution, self.config.xy_resolution),
            0.01)
        limit = max(1, math.ceil(position_tolerance / step))
        offsets = []
        for x_index in range(-limit, limit + 1):
            for y_index in range(-limit, limit + 1):
                dx = x_index * step
                dy = y_index * step
                distance = math.hypot(dx, dy)
                if distance <= position_tolerance + 1e-9:
                    offsets.append((distance, dx, dy))
        offsets.sort(key=lambda item: item[0])
        return tuple((dx, dy) for _, dx, dy in offsets)

    def _goal_yaw_candidates(self, yaw, require_goal_yaw):
        step = self.config.yaw_resolution
        limit = (
            self.config.goal_yaw_tolerance
            if require_goal_yaw
            else self.config.relaxed_goal_yaw_tolerance)
        count = max(0, math.ceil(limit / step))
        candidates = [(0.0, normalize_angle(yaw))]
        for index in range(1, count + 1):
            delta = min(limit, index * step)
            candidates.append((delta, normalize_angle(yaw + delta)))
            if delta < math.pi:
                candidates.append((delta, normalize_angle(yaw - delta)))
        candidates.sort(key=lambda item: item[0])
        unique = []
        seen = set()
        for _, candidate in candidates:
            key = round(candidate / step)
            if key in seen:
                continue
            seen.add(key)
            unique.append(candidate)
        return tuple(unique)

    def goal_region_has_collision_free_pose(
            self, goal, require_goal_yaw, position_tolerance):
        for dx, dy in self._goal_position_offsets(position_tolerance):
            for yaw in self._goal_yaw_candidates(goal.yaw, require_goal_yaw):
                candidate = Pose2D(goal.x + dx, goal.y + dy, yaw)
                if self.pose_is_collision_free(candidate):
                    return True
        return False

    def _start_position_offsets(self):
        tolerance = self.config.start_collision_tolerance
        if tolerance <= 0.0:
            return ()
        step = max(
            min(self.grid_map.resolution, self.config.xy_resolution),
            0.01)
        limit = max(1, math.ceil(tolerance / step))
        offsets = []
        for x_index in range(-limit, limit + 1):
            for y_index in range(-limit, limit + 1):
                dx = x_index * step
                dy = y_index * step
                distance = math.hypot(dx, dy)
                if 0.0 < distance <= tolerance + 1e-9:
                    offsets.append((distance, dx, dy))
        offsets.sort(key=lambda item: item[0])
        return tuple((distance, dx, dy) for distance, dx, dy in offsets)

    def nearest_collision_free_start(self, start):
        if self.pose_is_collision_free(start):
            return start, 0.0
        for distance, dx, dy in self._start_position_offsets():
            candidate = Pose2D(start.x + dx, start.y + dy, start.yaw)
            if self.pose_is_collision_free(candidate):
                return candidate, distance
        return None, None

    def _simulate_primitive(self, start, steer, direction):
        pose = start
        segment = []
        remaining = self.config.primitive_length
        while remaining > 1e-9:
            distance = min(self.config.integration_step, remaining)
            signed_distance = distance * direction
            yaw_mid = pose.yaw + (
                0.5 * signed_distance *
                math.tan(steer) / self.config.wheelbase)
            x = pose.x + signed_distance * math.cos(yaw_mid)
            y = pose.y + signed_distance * math.sin(yaw_mid)
            yaw = normalize_angle(
                pose.yaw +
                signed_distance *
                math.tan(steer) / self.config.wheelbase)
            pose = Pose2D(x, y, yaw)
            if not self.pose_is_collision_free(pose):
                return None
            segment.append(pose)
            remaining -= distance
        return segment

    def _reconstruct_path(self, closed, final_key):
        segments = []
        key = final_key
        while key is not None:
            node = closed[key]
            if node.segment:
                segments.append(node.segment)
            else:
                segments.append([node.pose])
            key = node.parent_key
        poses = []
        for segment in reversed(segments):
            if poses and segment:
                first = segment[0]
                previous = poses[-1]
                if math.hypot(
                        first.x - previous.x,
                        first.y - previous.y) < 1e-6:
                    segment = segment[1:]
            poses.extend(segment)
        return poses

    def plan(
            self,
            start,
            goal,
            require_goal_yaw=True,
            position_tolerance=None,
            approach_goal=False,
            deadline=None,
            should_cancel=None):
        begin = time.monotonic()
        if deadline is None and self.config.planning_timeout_sec > 0.0:
            deadline = begin + self.config.planning_timeout_sec
        position_tolerance = (
            self.config.goal_tolerance if position_tolerance is None
            else max(float(position_tolerance), self.config.xy_resolution))
        start = Pose2D(start.x, start.y, normalize_angle(start.yaw))
        start, start_adjustment_distance = (
            self.nearest_collision_free_start(start))
        if start is None:
            raise ValueError(
                'start pose collides with the map or map boundary; no '
                'collision-free nearby start within '
                f'{self.config.start_collision_tolerance:.3f}m')
        goal = Pose2D(goal.x, goal.y, normalize_angle(goal.yaw))
        if not self.goal_region_has_collision_free_pose(
                goal, require_goal_yaw, position_tolerance):
            raise ValueError('goal pose collides with the map or map boundary')

        start_key = self._state_key(start, 1)
        start_node = _SearchNode(
            pose=start,
            steer=0.0,
            direction=1,
            cost=0.0,
            parent_key=None,
            segment=[],
        )
        open_nodes = {start_key: start_node}
        closed = {}
        queue = []
        sequence = 0
        heapq.heappush(
            queue,
            (
                self._heuristic(start, goal, require_goal_yaw),
                sequence,
                start_key,
                start_node.cost,
            ))
        expansions = 0

        while queue:
            if should_cancel is not None and should_cancel():
                raise PlanningCancelled('planning cancelled')
            if expansions >= self.config.max_expansions:
                break
            if deadline is not None and time.monotonic() > deadline:
                break

            _, _, current_key, queued_cost = heapq.heappop(queue)
            current = open_nodes.get(current_key)
            if current is None or abs(current.cost - queued_cost) > 1e-9:
                continue
            open_nodes.pop(current_key)
            closed[current_key] = current
            expansions += 1

            if self._goal_reached(
                    current.pose, goal, require_goal_yaw, position_tolerance):
                return PlannedPath(
                    poses=self._reconstruct_path(closed, current_key),
                    cost=current.cost,
                    expansions=expansions,
                    planning_time_sec=time.monotonic() - begin,
                    relaxed_goal_yaw=not require_goal_yaw,
                    approach_goal=approach_goal,
                    requested_goal_distance=math.hypot(
                        current.pose.x - goal.x,
                        current.pose.y - goal.y),
                    requested_goal_yaw_error=normalize_angle(
                        goal.yaw - current.pose.yaw),
                    goal_tolerance=position_tolerance,
                    start_adjusted=start_adjustment_distance > 0.0,
                    start_adjustment_distance=start_adjustment_distance,
                )

            directions = (1, -1) if self.config.allow_reverse else (1,)
            for direction in directions:
                for steer in self.steering_inputs:
                    segment = self._simulate_primitive(
                        current.pose, steer, direction)
                    if not segment:
                        continue
                    next_pose = segment[-1]
                    next_key = self._state_key(next_pose, direction)
                    if next_key in closed:
                        continue

                    steer_ratio = abs(steer) / self.config.max_steer_angle
                    steer_change = abs(steer - current.steer) / \
                        self.config.max_steer_angle
                    travel_cost = self.config.primitive_length
                    if direction < 0:
                        travel_cost *= self.config.reverse_cost_multiplier
                    added_cost = (
                        travel_cost +
                        self.config.steer_cost * steer_ratio +
                        self.config.steer_change_cost * steer_change)
                    if direction != current.direction:
                        added_cost += self.config.direction_switch_cost
                    next_cost = current.cost + added_cost
                    existing = open_nodes.get(next_key)
                    if existing is not None and existing.cost <= next_cost:
                        continue

                    node = _SearchNode(
                        pose=next_pose,
                        steer=steer,
                        direction=direction,
                        cost=next_cost,
                        parent_key=current_key,
                        segment=segment,
                    )
                    open_nodes[next_key] = node
                    sequence += 1
                    priority = next_cost + self._heuristic(
                        next_pose, goal, require_goal_yaw)
                    heapq.heappush(
                        queue, (priority, sequence, next_key, next_cost))

        elapsed = time.monotonic() - begin
        raise RuntimeError(
            f'no path after {expansions} expansions in {elapsed:.3f}s')

    def plan_with_yaw_fallback(self, start, goal, should_cancel=None):
        errors = []
        overall_deadline = None
        if self.config.planning_timeout_sec > 0.0:
            overall_deadline = (
                time.monotonic() + self.config.planning_timeout_sec)

        def stage_deadline():
            deadlines = []
            if overall_deadline is not None:
                deadlines.append(overall_deadline)
            if self.config.planning_stage_timeout_sec > 0.0:
                deadlines.append(
                    time.monotonic() +
                    self.config.planning_stage_timeout_sec)
            if not deadlines:
                return None
            return min(deadlines)

        try:
            return self.plan(
                start,
                goal,
                require_goal_yaw=True,
                position_tolerance=self.config.goal_tolerance,
                approach_goal=False,
                deadline=stage_deadline(),
                should_cancel=should_cancel)
        except PlanningCancelled:
            raise
        except (RuntimeError, ValueError) as exc:
            errors.append(str(exc))

        if self.config.approach_goal_on_failure:
            try:
                return self.plan(
                    start,
                    goal,
                    require_goal_yaw=True,
                    position_tolerance=self.config.approach_goal_tolerance,
                    approach_goal=True,
                    deadline=stage_deadline(),
                    should_cancel=should_cancel)
            except PlanningCancelled:
                raise
            except (RuntimeError, ValueError) as exc:
                errors.append(str(exc))

        if self.config.allow_goal_yaw_fallback:
            try:
                return self.plan(
                    start,
                    goal,
                    require_goal_yaw=False,
                    position_tolerance=self.config.goal_tolerance,
                    approach_goal=False,
                    deadline=stage_deadline(),
                    should_cancel=should_cancel)
            except PlanningCancelled:
                raise
            except (RuntimeError, ValueError) as exc:
                errors.append(str(exc))
            if self.config.approach_goal_on_failure:
                try:
                    return self.plan(
                        start,
                        goal,
                        require_goal_yaw=False,
                        position_tolerance=(
                            self.config.approach_goal_tolerance),
                        approach_goal=True,
                        deadline=stage_deadline(),
                        should_cancel=should_cancel)
                except PlanningCancelled:
                    raise
                except (RuntimeError, ValueError) as exc:
                    errors.append(str(exc))

        if errors:
            raise RuntimeError(
                'no exact or approach path found: ' + ' | '.join(errors))
        raise RuntimeError('no exact or approach path found')
