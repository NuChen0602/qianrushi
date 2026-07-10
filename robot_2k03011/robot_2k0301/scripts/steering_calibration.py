#!/usr/bin/env python3
import argparse
import math
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = PROJECT_ROOT / 'ros2_ws' / 'src' / 'robot_lidar_bridge'
sys.path.insert(0, str(PACKAGE_ROOT))

from robot_lidar_bridge.motion_control import SteeringCalibration  # noqa: E402


def parse_point(value):
    fields = value.split(':')
    if len(fields) != 3:
        raise argparse.ArgumentTypeError(
            'point must use COMMAND:SERVO_DEG:WHEEL_DEG')
    try:
        point = tuple(float(field) for field in fields)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            'point contains a non-number') from exc
    if not all(math.isfinite(field) for field in point):
        raise argparse.ArgumentTypeError('point values must be finite')
    return point


def format_array(values):
    return '[' + ', '.join(f'{value:.3f}' for value in values) + ']'


def main():
    parser = argparse.ArgumentParser(
        description='Generate steering calibration YAML from measurements.')
    parser.add_argument(
        '--point',
        action='append',
        required=True,
        type=parse_point,
        help='COMMAND:SERVO_DEG:WHEEL_DEG; provide at least three points')
    parser.add_argument('--wheelbase', type=float, default=0.18)
    args = parser.parse_args()

    if len(args.point) < 3:
        parser.error('at least three --point measurements are required')
    points = sorted(args.point, key=lambda point: point[0])
    calibration = SteeringCalibration(
        command_points=tuple(point[0] for point in points),
        servo_deg_points=tuple(point[1] for point in points),
        wheel_deg_points=tuple(point[2] for point in points),
        wheelbase_m=args.wheelbase,
    )

    print(
        'Paste these values under '
        'robot_odometry_tcp_bridge.ros__parameters:')
    print(f'wheelbase_m: {args.wheelbase:.3f}')
    print('steering_command_points: ' + format_array(
        calibration.command_points))
    print('steering_servo_deg_points: ' + format_array(
        calibration.servo_deg_points))
    print('steering_wheel_deg_points: ' + format_array(
        calibration.wheel_deg_points))
    print('\nMeasured geometry:')
    for command, servo, wheel in points:
        radius = calibration.turning_radius_m(command)
        radius_text = 'straight' if not math.isfinite(radius) else \
            f'{radius:.3f} m'
        print(
            f'  command={command:+.3f} servo={servo:.2f} deg '
            f'wheel={wheel:+.2f} deg radius={radius_text}')


if __name__ == '__main__':
    main()
