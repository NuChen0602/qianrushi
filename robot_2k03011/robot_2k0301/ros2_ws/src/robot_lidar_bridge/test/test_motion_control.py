import math
import unittest

from robot_lidar_bridge.motion_control import (
    SmoothSpeedLimiter,
    SteeringCalibration,
    curvature_steering_command,
    curvature_speed_limit,
    peak_path_curvature,
    signed_path_curvature,
)


class MotionControlTest(unittest.TestCase):
    def test_signed_curvature_and_ackermann_feedforward(self):
        left_path = [
            (0.0, 0.0, 0.0),
            (0.08, 0.0, 0.20),
            (0.16, 0.02, 0.40),
            (0.23, 0.05, 0.60),
        ]
        right_path = [(x, -y, -yaw) for x, y, yaw in left_path]

        left = signed_path_curvature(left_path, 0, 0.24)
        right = signed_path_curvature(right_path, 0, 0.24)
        self.assertGreater(left, 0.0)
        self.assertLess(right, 0.0)
        self.assertAlmostEqual(left, -right, places=6)
        self.assertGreater(
            curvature_steering_command(left, 0.18, 28.0), 0.0)

    def test_curvature_lookahead_slows_before_turn(self):
        path = [
            (0.00, 0.00, 0.0),
            (0.10, 0.00, 0.0),
            (0.20, 0.00, 0.0),
            (0.29, 0.03, 0.35),
            (0.36, 0.10, 0.75),
        ]
        curvature = peak_path_curvature(path, 0, 0.45)
        self.assertGreater(curvature, 2.0)
        limit = curvature_speed_limit(0.18, 0.07, curvature, 0.45)
        self.assertLess(limit, 0.12)

    def test_speed_limiter_respects_acceleration_and_reversal(self):
        limiter = SmoothSpeedLimiter(0.20, 0.30, 1.0)
        values = [limiter.update(0.18, 0.05) for _ in range(20)]
        self.assertTrue(all(
            second >= first for first, second in zip(values, values[1:])))
        self.assertLessEqual(max(values), 0.18)
        self.assertLessEqual(limiter.acceleration, 0.20)

        previous = limiter.velocity
        reversed_velocity = limiter.update(-0.10, 0.05)
        self.assertGreaterEqual(reversed_velocity, 0.0)
        self.assertLess(reversed_velocity, previous)

    def test_piecewise_steering_calibration_and_radius(self):
        calibration = SteeringCalibration(
            command_points=(-1.0, -0.5, 0.0, 0.5, 1.0),
            servo_deg_points=(80.0, 87.5, 95.0, 107.5, 120.0),
            wheel_deg_points=(-35.0, -16.0, 0.0, 18.0, 36.0),
            wheelbase_m=0.18,
        )
        self.assertAlmostEqual(calibration.servo_deg(0.25), 101.25)
        self.assertAlmostEqual(calibration.wheel_deg(-0.75), -25.5)
        self.assertAlmostEqual(
            calibration.command_for_servo_deg(101.25), 0.25)
        self.assertAlmostEqual(
            calibration.wheel_deg_for_servo_deg(101.25), 9.0)
        self.assertTrue(math.isinf(calibration.turning_radius_m(0.0)))
        self.assertGreater(calibration.turning_radius_m(0.5), 0.0)


if __name__ == '__main__':
    unittest.main()
