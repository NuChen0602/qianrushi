import math
import unittest

from robot_lidar_bridge.ackermann_a_star import (
    HybridAStarPlanner,
    OccupancyGridMap,
    PlannerConfig,
    Pose2D,
)


def make_test_map():
    resolution = 0.02
    width = 100
    height = 100
    data = [0] * (width * height)
    for x in range(width):
        data[x] = 100
        data[(height - 1) * width + x] = 100
    for y in range(height):
        data[y * width] = 100
        data[y * width + width - 1] = 100
    for y in range(1, 72):
        if 42 <= y <= 58:
            continue
        for x in range(48, 52):
            data[y * width + x] = 100
    return OccupancyGridMap(
        width=width,
        height=height,
        resolution=resolution,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=0.0,
        data=data,
    )


class HybridAStarPlannerTest(unittest.TestCase):
    def setUp(self):
        config = PlannerConfig(
            planning_timeout_sec=6.0,
            max_expansions=120000,
            reverse_cost_multiplier=1.4,
            direction_switch_cost=0.4,
        )
        self.planner = HybridAStarPlanner(make_test_map(), config)

    def test_path_respects_ackermann_motion_and_obstacles(self):
        start = Pose2D(0.30, 0.40, 0.0)
        goal = Pose2D(1.15, 0.90, 0.0)
        result = self.planner.plan_with_yaw_fallback(start, goal)

        self.assertGreater(len(result.poses), 5)
        self.assertLess(
            math.hypot(
                result.poses[-1].x - goal.x,
                result.poses[-1].y - goal.y),
            self.planner.config.goal_tolerance + 1e-6)
        self.assertTrue(self.planner.segment_is_collision_free(result.poses))
        self.assertFalse(result.relaxed_goal_yaw)
        final_yaw_error = math.atan2(
            math.sin(result.poses[-1].yaw - goal.yaw),
            math.cos(result.poses[-1].yaw - goal.yaw))
        self.assertLessEqual(
            abs(final_yaw_error),
            self.planner.config.goal_yaw_tolerance + 1e-6)

        max_yaw_step = (
            self.planner.config.integration_step *
            math.tan(self.planner.config.max_steer_angle) /
            self.planner.config.wheelbase)
        for first, second in zip(result.poses, result.poses[1:]):
            yaw_step = abs(math.atan2(
                math.sin(second.yaw - first.yaw),
                math.cos(second.yaw - first.yaw)))
            self.assertLessEqual(yaw_step, max_yaw_step + 1e-6)

    def test_rejects_goal_inside_obstacle_when_approach_is_disabled(self):
        with self.assertRaises(ValueError):
            self.planner.plan(
                Pose2D(0.30, 0.40, 0.0),
                Pose2D(1.00, 0.50, 0.0))

    def test_accepts_goal_when_nearby_parking_pose_is_free(self):
        resolution = 0.02
        width = 80
        height = 80
        data = [0] * (width * height)
        goal_x = 1.00
        goal_y = 1.00
        gx = int(goal_x / resolution)
        gy = int(goal_y / resolution)
        data[gy * width + gx] = 100
        grid_map = OccupancyGridMap(
            width=width,
            height=height,
            resolution=resolution,
            origin_x=0.0,
            origin_y=0.0,
            origin_yaw=0.0,
            data=data,
        )
        planner = HybridAStarPlanner(
            grid_map,
            PlannerConfig(
                vehicle_length=0.04,
                vehicle_width=0.04,
                safety_margin=0.0,
                goal_tolerance=0.10,
                planning_timeout_sec=5.0,
                max_expansions=120000,
            ))

        goal = Pose2D(goal_x, goal_y, 0.0)
        result = planner.plan_with_yaw_fallback(
            Pose2D(0.30, 1.00, 0.0),
            goal)

        self.assertLessEqual(
            math.hypot(
                result.poses[-1].x - goal.x,
                result.poses[-1].y - goal.y),
            planner.config.goal_tolerance + 1e-6)
        self.assertTrue(planner.segment_is_collision_free(result.poses))

    def test_adjusts_start_when_pose_slightly_overlaps_map(self):
        resolution = 0.02
        width = 80
        height = 80
        data = [0] * (width * height)
        start_x = 0.30
        start_y = 1.00
        start_gx = int(start_x / resolution)
        start_gy = int(start_y / resolution)
        data[start_gy * width + start_gx] = 100
        grid_map = OccupancyGridMap(
            width=width,
            height=height,
            resolution=resolution,
            origin_x=0.0,
            origin_y=0.0,
            origin_yaw=0.0,
            data=data,
        )
        planner = HybridAStarPlanner(
            grid_map,
            PlannerConfig(
                vehicle_length=0.04,
                vehicle_width=0.04,
                safety_margin=0.0,
                start_collision_tolerance=0.08,
                planning_timeout_sec=5.0,
                max_expansions=120000,
            ))

        result = planner.plan_with_yaw_fallback(
            Pose2D(start_x, start_y, 0.0),
            Pose2D(0.80, 1.00, 0.0))

        self.assertTrue(result.start_adjusted)
        self.assertGreater(result.start_adjustment_distance, 0.0)
        self.assertLessEqual(result.start_adjustment_distance, 0.08)
        self.assertTrue(planner.segment_is_collision_free(result.poses))

    def test_approaches_goal_when_exact_pose_is_blocked(self):
        resolution = 0.02
        width = 80
        height = 80
        data = [0] * (width * height)
        goal_x = 1.00
        goal_y = 1.00
        goal_gx = int(goal_x / resolution)
        goal_gy = int(goal_y / resolution)
        for gy in range(goal_gy - 4, goal_gy + 5):
            for gx in range(goal_gx - 4, goal_gx + 5):
                data[gy * width + gx] = 100
        grid_map = OccupancyGridMap(
            width=width,
            height=height,
            resolution=resolution,
            origin_x=0.0,
            origin_y=0.0,
            origin_yaw=0.0,
            data=data,
        )
        planner = HybridAStarPlanner(
            grid_map,
            PlannerConfig(
                vehicle_length=0.04,
                vehicle_width=0.04,
                safety_margin=0.0,
                goal_tolerance=0.04,
                approach_goal_tolerance=0.30,
                approach_goal_on_failure=True,
                allow_goal_yaw_fallback=True,
                planning_timeout_sec=5.0,
                max_expansions=120000,
            ))

        result = planner.plan_with_yaw_fallback(
            Pose2D(0.25, 1.00, 0.0),
            Pose2D(goal_x, goal_y, 0.0))

        self.assertTrue(result.approach_goal)
        self.assertFalse(result.relaxed_goal_yaw)
        self.assertGreater(result.requested_goal_distance, 0.04)
        self.assertLessEqual(result.requested_goal_distance, 0.30 + 1e-6)
        self.assertTrue(planner.segment_is_collision_free(result.poses))

    def test_relaxed_yaw_fallback_still_limits_final_heading(self):
        resolution = 0.02
        width = 120
        height = 120
        data = [0] * (width * height)
        for x in range(width):
            data[x] = 100
            data[(height - 1) * width + x] = 100
        for y in range(height):
            data[y * width] = 100
            data[y * width + width - 1] = 100
        grid_map = OccupancyGridMap(
            width=width,
            height=height,
            resolution=resolution,
            origin_x=0.0,
            origin_y=0.0,
            origin_yaw=0.0,
            data=data,
        )
        planner = HybridAStarPlanner(
            grid_map,
            PlannerConfig(
                vehicle_length=0.20,
                vehicle_width=0.10,
                safety_margin=0.0,
                goal_tolerance=0.10,
                goal_yaw_tolerance=math.radians(10.0),
                relaxed_goal_yaw_tolerance=math.radians(45.0),
                planning_timeout_sec=5.0,
                max_expansions=120000,
                reverse_cost_multiplier=1.4,
                direction_switch_cost=0.4,
            ))

        result = planner.plan(
            Pose2D(0.60, 0.60, math.radians(90.0)),
            Pose2D(0.62, 0.60, 0.0),
            require_goal_yaw=False)

        self.assertTrue(result.relaxed_goal_yaw)
        self.assertGreater(len(result.poses), 1)
        self.assertLessEqual(
            abs(result.requested_goal_yaw_error),
            planner.config.relaxed_goal_yaw_tolerance + 1e-6)
        self.assertTrue(planner.segment_is_collision_free(result.poses))

    def test_can_reverse_to_a_goal_behind_the_car(self):
        start = Pose2D(0.55, 1.55, 0.0)
        goal = Pose2D(0.25, 1.55, 0.0)
        result = self.planner.plan_with_yaw_fallback(start, goal)
        self.assertLess(
            math.hypot(
                result.poses[-1].x - goal.x,
                result.poses[-1].y - goal.y),
            self.planner.config.goal_tolerance + 1e-6)
        first = result.poses[0]
        second = result.poses[1]
        projection = (
            (second.x - first.x) * math.cos(first.yaw) +
            (second.y - first.y) * math.sin(first.yaw))
        self.assertLess(projection, 0.0)


if __name__ == '__main__':
    unittest.main()
