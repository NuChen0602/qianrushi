import math
from dataclasses import dataclass


@dataclass(frozen=True)
class GridSpec:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float = 0.0

    def __post_init__(self):
        if self.width <= 0 or self.height <= 0:
            raise ValueError('grid dimensions must be positive')
        if self.resolution <= 0.0:
            raise ValueError('grid resolution must be positive')

    def world_to_local(self, x, y):
        dx = x - self.origin_x
        dy = y - self.origin_y
        cosine = math.cos(self.origin_yaw)
        sine = math.sin(self.origin_yaw)
        return cosine * dx + sine * dy, -sine * dx + cosine * dy

    def local_to_world(self, x, y):
        cosine = math.cos(self.origin_yaw)
        sine = math.sin(self.origin_yaw)
        return (
            self.origin_x + cosine * x - sine * y,
            self.origin_y + sine * x + cosine * y,
        )

    def world_to_grid(self, x, y):
        local_x, local_y = self.world_to_local(x, y)
        return (
            math.floor(local_x / self.resolution),
            math.floor(local_y / self.resolution),
        )

    def grid_to_world(self, gx, gy):
        return self.local_to_world(
            (gx + 0.5) * self.resolution,
            (gy + 0.5) * self.resolution)

    def contains(self, gx, gy):
        return 0 <= gx < self.width and 0 <= gy < self.height


class StaticGrid:
    def __init__(self, spec, data, occupied_threshold=65):
        if len(data) != spec.width * spec.height:
            raise ValueError('occupancy data size does not match grid')
        self.spec = spec
        self.data = tuple(int(value) for value in data)
        self.occupied_threshold = int(occupied_threshold)

    def occupied(self, gx, gy):
        if not self.spec.contains(gx, gy):
            return True
        return self.data[gy * self.spec.width + gx] >= self.occupied_threshold

    def occupied_near_world(self, x, y, radius):
        local_x, local_y = self.spec.world_to_local(x, y)
        center_x = math.floor(local_x / self.spec.resolution)
        center_y = math.floor(local_y / self.spec.resolution)
        cells = max(0, math.ceil(radius / self.spec.resolution))
        radius_squared = radius * radius
        for gy in range(center_y - cells, center_y + cells + 1):
            for gx in range(center_x - cells, center_x + cells + 1):
                if not self.occupied(gx, gy):
                    continue
                cell_min_x = gx * self.spec.resolution
                cell_max_x = cell_min_x + self.spec.resolution
                cell_min_y = gy * self.spec.resolution
                cell_max_y = cell_min_y + self.spec.resolution
                nearest_x = max(cell_min_x, min(cell_max_x, local_x))
                nearest_y = max(cell_min_y, min(cell_max_y, local_y))
                if ((nearest_x - local_x) ** 2 +
                        (nearest_y - local_y) ** 2 <= radius_squared):
                    return True
        return False

    def inflated_data(self, dynamic_cells, inflation_radius):
        result = list(self.data)
        radius_cells = max(
            0, math.ceil(inflation_radius / self.spec.resolution))
        radius_squared = inflation_radius * inflation_radius
        for center_x, center_y in dynamic_cells:
            if not self.spec.contains(center_x, center_y):
                continue
            for gy in range(center_y - radius_cells,
                            center_y + radius_cells + 1):
                for gx in range(center_x - radius_cells,
                                center_x + radius_cells + 1):
                    if not self.spec.contains(gx, gy):
                        continue
                    dx = (gx - center_x) * self.spec.resolution
                    dy = (gy - center_y) * self.spec.resolution
                    if dx * dx + dy * dy <= radius_squared + 1e-12:
                        result[gy * self.spec.width + gx] = 100
        return result


@dataclass(frozen=True)
class LocalizationThresholds:
    pose_timeout_sec: float = 2.0
    scan_timeout_sec: float = 0.8
    odom_timeout_sec: float = 0.8
    warn_xy_std_m: float = 0.12
    fail_xy_std_m: float = 0.25
    warn_yaw_std_rad: float = math.radians(15.0)
    fail_yaw_std_rad: float = math.radians(30.0)


def evaluate_localization(
        covariance, pose_age, scan_age, odom_age, thresholds=None):
    limits = thresholds or LocalizationThresholds()
    ages = {
        'pose': float(pose_age),
        'scan': float(scan_age),
        'odom': float(odom_age),
    }
    timeouts = {
        'pose': limits.pose_timeout_sec,
        'scan': limits.scan_timeout_sec,
        'odom': limits.odom_timeout_sec,
    }
    for source in ('pose', 'scan', 'odom'):
        if not math.isfinite(ages[source]) or ages[source] > timeouts[source]:
            return {
                'state': 'lost',
                'ok': False,
                'reason': f'{source}_timeout',
                'quality': 0.0,
                'pose_age_sec': ages['pose'],
                'scan_age_sec': ages['scan'],
                'odom_age_sec': ages['odom'],
            }

    try:
        variance_x = float(covariance[0])
        variance_y = float(covariance[7])
        variance_yaw = float(covariance[35])
    except (IndexError, TypeError, ValueError):
        variance_x = variance_y = variance_yaw = math.nan
    if (not all(math.isfinite(value) for value in (
            variance_x, variance_y, variance_yaw)) or
            min(variance_x, variance_y, variance_yaw) < 0.0):
        return {
            'state': 'lost',
            'ok': False,
            'reason': 'invalid_covariance',
            'quality': 0.0,
            'pose_age_sec': ages['pose'],
            'scan_age_sec': ages['scan'],
            'odom_age_sec': ages['odom'],
        }

    xy_std = math.sqrt(max(variance_x, variance_y))
    yaw_std = math.sqrt(variance_yaw)
    if xy_std > limits.fail_xy_std_m:
        state, reason = 'lost', 'position_uncertain'
    elif yaw_std > limits.fail_yaw_std_rad:
        state, reason = 'lost', 'heading_uncertain'
    elif (xy_std > limits.warn_xy_std_m or
          yaw_std > limits.warn_yaw_std_rad):
        state, reason = 'degraded', 'covariance_elevated'
    else:
        state, reason = 'good', ''

    position_ratio = min(1.0, xy_std / limits.fail_xy_std_m)
    heading_ratio = min(1.0, yaw_std / limits.fail_yaw_std_rad)
    quality = max(0.0, 100.0 * (
        1.0 - 0.65 * position_ratio - 0.35 * heading_ratio))
    return {
        'state': state,
        'ok': state != 'lost',
        'reason': reason,
        'quality': quality,
        'xy_std_m': xy_std,
        'yaw_std_deg': math.degrees(yaw_std),
        'pose_age_sec': ages['pose'],
        'scan_age_sec': ages['scan'],
        'odom_age_sec': ages['odom'],
    }


def _point_segment_distance(px, py, ax, ay, bx, by):
    dx = bx - ax
    dy = by - ay
    length_squared = dx * dx + dy * dy
    if length_squared <= 1e-12:
        return math.hypot(px - ax, py - ay)
    ratio = ((px - ax) * dx + (py - ay) * dy) / length_squared
    ratio = max(0.0, min(1.0, ratio))
    nearest_x = ax + ratio * dx
    nearest_y = ay + ratio * dy
    return math.hypot(px - nearest_x, py - nearest_y)


def path_blockage(
        dynamic_points, path_points, robot_x, robot_y,
        lookahead_distance, corridor_radius):
    if not dynamic_points or not path_points:
        return False, math.inf, 0

    nearest_index = min(
        range(len(path_points)),
        key=lambda index: math.hypot(
            path_points[index][0] - robot_x,
            path_points[index][1] - robot_y))
    selected = [(robot_x, robot_y)]
    distance = 0.0
    previous = selected[0]
    for point in path_points[nearest_index:]:
        current = (point[0], point[1])
        distance += math.hypot(
            current[0] - previous[0], current[1] - previous[1])
        selected.append(current)
        previous = current
        if distance >= lookahead_distance:
            break

    nearest_distance = math.inf
    blocking_points = 0
    for px, py in dynamic_points:
        distance_to_path = min(
            _point_segment_distance(px, py, *first, *second)
            for first, second in zip(selected, selected[1:]))
        nearest_distance = min(nearest_distance, distance_to_path)
        if distance_to_path <= corridor_radius:
            blocking_points += 1
    return blocking_points > 0, nearest_distance, blocking_points
