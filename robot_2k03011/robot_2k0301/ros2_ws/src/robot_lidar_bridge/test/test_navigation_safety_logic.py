import math
import unittest

from robot_lidar_bridge.navigation_safety_logic import (
    GridSpec,
    LocalizationThresholds,
    StaticGrid,
    evaluate_localization,
    path_blockage,
)


class NavigationSafetyLogicTest(unittest.TestCase):
    def setUp(self):
        self.spec = GridSpec(
            width=20,
            height=20,
            resolution=0.05,
            origin_x=0.0,
            origin_y=0.0,
        )
        data = [0] * (self.spec.width * self.spec.height)
        for y in range(self.spec.height):
            data[y * self.spec.width + 10] = 100
        self.grid = StaticGrid(self.spec, data)

    def test_static_wall_filter_and_dynamic_inflation(self):
        self.assertTrue(self.grid.occupied_near_world(0.48, 0.5, 0.04))
        self.assertFalse(self.grid.occupied_near_world(0.30, 0.5, 0.04))

        dynamic_cell = self.spec.world_to_grid(0.30, 0.50)
        combined = self.grid.inflated_data([dynamic_cell], 0.08)
        center_x, center_y = dynamic_cell
        self.assertEqual(
            combined[center_y * self.spec.width + center_x], 100)
        self.assertEqual(
            combined[center_y * self.spec.width + center_x + 1], 100)

    def test_path_blockage_only_checks_path_corridor(self):
        path = [(0.10, 0.20), (0.40, 0.20), (0.80, 0.20)]
        blocked, distance, count = path_blockage(
            [(0.35, 0.24), (0.35, 0.60)],
            path,
            0.10,
            0.20,
            lookahead_distance=0.60,
            corridor_radius=0.10,
        )
        self.assertTrue(blocked)
        self.assertLess(distance, 0.05)
        self.assertEqual(count, 1)

        blocked, _, count = path_blockage(
            [(0.35, 0.60)],
            path,
            0.10,
            0.20,
            lookahead_distance=0.60,
            corridor_radius=0.10,
        )
        self.assertFalse(blocked)
        self.assertEqual(count, 0)

    def test_localization_quality_uses_covariance_and_freshness(self):
        covariance = [0.0] * 36
        covariance[0] = 0.05 ** 2
        covariance[7] = 0.06 ** 2
        covariance[35] = math.radians(5.0) ** 2
        result = evaluate_localization(
            covariance, 0.1, 0.1, 0.1, LocalizationThresholds())
        self.assertEqual(result['state'], 'good')
        self.assertTrue(result['ok'])

        covariance[0] = 0.30 ** 2
        result = evaluate_localization(
            covariance, 0.1, 0.1, 0.1, LocalizationThresholds())
        self.assertEqual(result['state'], 'lost')
        self.assertEqual(result['reason'], 'position_uncertain')

        result = evaluate_localization(
            covariance, 3.0, 0.1, 0.1, LocalizationThresholds())
        self.assertEqual(result['reason'], 'pose_timeout')


if __name__ == '__main__':
    unittest.main()
