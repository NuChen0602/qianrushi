import math
from dataclasses import dataclass


def clamp(value, low, high):
    return max(low, min(high, value))


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def piecewise_linear(value, input_points, output_points):
    if len(input_points) != len(output_points) or len(input_points) < 2:
        raise ValueError('calibration tables must have equal lengths >= 2')
    inputs = [float(point) for point in input_points]
    outputs = [float(point) for point in output_points]
    if not all(math.isfinite(point) for point in inputs + outputs):
        raise ValueError('calibration table contains non-finite values')
    if any(
            second <= first
            for first, second in zip(inputs, inputs[1:])):
        raise ValueError(
            'calibration input points must be strictly increasing')

    target = clamp(float(value), inputs[0], inputs[-1])
    for index in range(len(inputs) - 1):
        lower = inputs[index]
        upper = inputs[index + 1]
        if target <= upper:
            ratio = (target - lower) / (upper - lower)
            return outputs[index] + ratio * (
                outputs[index + 1] - outputs[index])
    return outputs[-1]


@dataclass(frozen=True)
class SteeringCalibration:
    command_points: tuple
    servo_deg_points: tuple
    wheel_deg_points: tuple
    wheelbase_m: float

    def __post_init__(self):
        piecewise_linear(0.0, self.command_points, self.servo_deg_points)
        piecewise_linear(0.0, self.command_points, self.wheel_deg_points)
        piecewise_linear(
            self.servo_deg_points[0],
            self.servo_deg_points,
            self.command_points)
        if self.wheelbase_m <= 0.0:
            raise ValueError('wheelbase must be positive')

    def servo_deg(self, command):
        return piecewise_linear(
            command, self.command_points, self.servo_deg_points)

    def wheel_deg(self, command):
        return piecewise_linear(
            command, self.command_points, self.wheel_deg_points)

    def command_for_servo_deg(self, servo_deg):
        return piecewise_linear(
            servo_deg, self.servo_deg_points, self.command_points)

    def wheel_deg_for_servo_deg(self, servo_deg):
        return self.wheel_deg(self.command_for_servo_deg(servo_deg))

    def turning_radius_m(self, command):
        wheel_angle = math.radians(self.wheel_deg(command))
        tangent = math.tan(wheel_angle)
        if abs(tangent) < 1e-6:
            return math.inf
        return self.wheelbase_m / tangent


def peak_path_curvature(path, start_index, lookahead_distance):
    if len(path) < 2:
        return 0.0
    begin = max(0, min(int(start_index), len(path) - 2))
    distance = 0.0
    samples = []
    for index in range(begin, len(path) - 1):
        first = path[index]
        second = path[index + 1]
        segment = math.hypot(second[0] - first[0], second[1] - first[1])
        if segment > 1e-5:
            yaw_change = abs(normalize_angle(second[2] - first[2]))
            samples.append(yaw_change / segment)
            distance += segment
        if distance >= lookahead_distance:
            break
    if not samples:
        return 0.0
    samples.sort(reverse=True)
    count = min(3, len(samples))
    return sum(samples[:count]) / count


def signed_path_curvature(path, start_index, lookahead_distance):
    """Estimate bicycle-model curvature along the current motion direction."""
    if len(path) < 2:
        return 0.0
    begin = max(0, min(int(start_index), len(path) - 2))
    distance = 0.0
    yaw_change = 0.0
    motion_direction = 0
    for index in range(begin, len(path) - 1):
        first = path[index]
        second = path[index + 1]
        dx = second[0] - first[0]
        dy = second[1] - first[1]
        segment = math.hypot(dx, dy)
        if segment <= 1e-5:
            continue
        projection = dx * math.cos(first[2]) + dy * math.sin(first[2])
        direction = 1 if projection >= 0.0 else -1
        if motion_direction == 0:
            motion_direction = direction
        elif direction != motion_direction:
            break
        yaw_change += normalize_angle(second[2] - first[2])
        distance += segment
        if distance >= lookahead_distance:
            break
    if distance <= 1e-5 or motion_direction == 0:
        return 0.0
    return yaw_change / (motion_direction * distance)


def curvature_steering_command(
        curvature, wheelbase_m, maximum_wheel_angle_deg):
    """Convert path curvature to normalized Ackermann steering command."""
    maximum_angle = math.radians(max(abs(maximum_wheel_angle_deg), 1.0))
    wheel_angle = math.atan(float(wheelbase_m) * float(curvature))
    return clamp(wheel_angle / maximum_angle, -1.0, 1.0)


def curvature_speed_limit(
        maximum_speed, minimum_curve_speed, curvature, curvature_gain):
    maximum = max(0.0, float(maximum_speed))
    minimum = clamp(float(minimum_curve_speed), 0.0, maximum)
    gain = max(0.0, float(curvature_gain))
    curve = max(0.0, abs(float(curvature)))
    return max(minimum, maximum / (1.0 + gain * curve))


class SmoothSpeedLimiter:
    def __init__(self, max_acceleration, max_deceleration, max_jerk):
        self.max_acceleration = max(float(max_acceleration), 0.01)
        self.max_deceleration = max(float(max_deceleration), 0.01)
        self.max_jerk = max(float(max_jerk), 0.01)
        self.velocity = 0.0
        self.acceleration = 0.0

    def reset(self, velocity=0.0):
        self.velocity = float(velocity)
        self.acceleration = 0.0

    def update(self, target_velocity, dt):
        dt = clamp(float(dt), 0.001, 0.2)
        target = float(target_velocity)
        if self.velocity * target < 0.0 and abs(self.velocity) > 1e-3:
            target = 0.0

        speeding_up = (
            self.velocity * target >= 0.0 and
            abs(target) > abs(self.velocity))
        acceleration_limit = (
            self.max_acceleration if speeding_up else self.max_deceleration)
        desired_acceleration = clamp(
            (target - self.velocity) / dt,
            -acceleration_limit,
            acceleration_limit)
        acceleration_delta = self.max_jerk * dt
        self.acceleration += clamp(
            desired_acceleration - self.acceleration,
            -acceleration_delta,
            acceleration_delta)
        self.acceleration = clamp(
            self.acceleration, -acceleration_limit, acceleration_limit)

        next_velocity = self.velocity + self.acceleration * dt
        if ((target - self.velocity) * (target - next_velocity) <= 0.0 or
                abs(target - next_velocity) < 1e-5):
            next_velocity = target
            self.acceleration = 0.0
        self.velocity = next_velocity
        return self.velocity
