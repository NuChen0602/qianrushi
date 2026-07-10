import json
import math
import threading

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, String
from tf2_ros import Buffer, TransformException, TransformListener

from robot_lidar_bridge.ackermann_a_star import (
    HybridAStarPlanner,
    OccupancyGridMap,
    PlanningCancelled,
    PlannerConfig,
    Pose2D,
)


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_quaternion(yaw):
    return 0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw)


class AckermannPathPlanner(Node):
    def __init__(self):
        super().__init__('ackermann_path_planner')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('path_topic', '/planned_path')
        self.declare_parameter('replan_topic', '/navigation/replan')
        self.declare_parameter('cancel_topic', '/navigation/cancel')
        self.declare_parameter('status_topic', '/planner/status')
        self.declare_parameter('occupied_threshold', 65)
        self.declare_parameter('unknown_is_occupied', True)
        self.declare_parameter('xy_resolution', 0.04)
        self.declare_parameter('yaw_resolution_deg', 10.0)
        self.declare_parameter('primitive_length', 0.08)
        self.declare_parameter('integration_step', 0.02)
        self.declare_parameter('wheelbase', 0.18)
        self.declare_parameter('vehicle_length', 0.26)
        self.declare_parameter('vehicle_width', 0.135)
        self.declare_parameter('safety_margin', 0.025)
        self.declare_parameter('max_steer_angle_deg', 37.0)
        self.declare_parameter('steering_samples', 5)
        self.declare_parameter('goal_tolerance', 0.10)
        self.declare_parameter('goal_yaw_tolerance_deg', 10.0)
        self.declare_parameter('relaxed_goal_yaw_tolerance_deg', 45.0)
        self.declare_parameter('start_collision_tolerance', 0.08)
        self.declare_parameter('approach_goal_on_failure', True)
        self.declare_parameter('approach_goal_tolerance', 0.35)
        self.declare_parameter('allow_goal_yaw_fallback', True)
        self.declare_parameter('allow_reverse', True)
        self.declare_parameter('reverse_cost_multiplier', 1.8)
        self.declare_parameter('direction_switch_cost', 0.8)
        self.declare_parameter('heading_heuristic_weight', 0.08)
        self.declare_parameter('relaxed_heading_heuristic_weight', 0.12)
        self.declare_parameter('max_expansions', 80000)
        self.declare_parameter('planning_timeout_sec', 3.0)
        self.declare_parameter('planning_stage_timeout_sec', 0.0)

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.occupied_threshold = int(
            self.get_parameter('occupied_threshold').value)
        self.unknown_is_occupied = bool(
            self.get_parameter('unknown_is_occupied').value)
        self.planner_config = PlannerConfig(
            xy_resolution=float(
                self.get_parameter('xy_resolution').value),
            yaw_resolution=math.radians(float(
                self.get_parameter('yaw_resolution_deg').value)),
            primitive_length=float(
                self.get_parameter('primitive_length').value),
            integration_step=float(
                self.get_parameter('integration_step').value),
            wheelbase=float(self.get_parameter('wheelbase').value),
            vehicle_length=float(
                self.get_parameter('vehicle_length').value),
            vehicle_width=float(
                self.get_parameter('vehicle_width').value),
            safety_margin=float(
                self.get_parameter('safety_margin').value),
            max_steer_angle=math.radians(float(
                self.get_parameter('max_steer_angle_deg').value)),
            steering_samples=int(
                self.get_parameter('steering_samples').value),
            goal_tolerance=float(
                self.get_parameter('goal_tolerance').value),
            goal_yaw_tolerance=math.radians(float(
                self.get_parameter('goal_yaw_tolerance_deg').value)),
            relaxed_goal_yaw_tolerance=math.radians(float(
                self.get_parameter(
                    'relaxed_goal_yaw_tolerance_deg').value)),
            start_collision_tolerance=float(
                self.get_parameter('start_collision_tolerance').value),
            approach_goal_on_failure=bool(
                self.get_parameter('approach_goal_on_failure').value),
            approach_goal_tolerance=float(
                self.get_parameter('approach_goal_tolerance').value),
            allow_goal_yaw_fallback=bool(
                self.get_parameter('allow_goal_yaw_fallback').value),
            allow_reverse=bool(
                self.get_parameter('allow_reverse').value),
            reverse_cost_multiplier=float(
                self.get_parameter('reverse_cost_multiplier').value),
            direction_switch_cost=float(
                self.get_parameter('direction_switch_cost').value),
            heading_heuristic_weight=float(
                self.get_parameter('heading_heuristic_weight').value),
            relaxed_heading_heuristic_weight=float(
                self.get_parameter(
                    'relaxed_heading_heuristic_weight').value),
            max_expansions=int(
                self.get_parameter('max_expansions').value),
            planning_timeout_sec=float(
                self.get_parameter('planning_timeout_sec').value),
            planning_stage_timeout_sec=float(
                self.get_parameter('planning_stage_timeout_sec').value),
        ).sanitized()

        transient_qos = QoSProfile(depth=1)
        transient_qos.reliability = ReliabilityPolicy.RELIABLE
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.path_publisher = self.create_publisher(
            Path,
            str(self.get_parameter('path_topic').value),
            transient_qos)
        self.status_publisher = self.create_publisher(
            String,
            str(self.get_parameter('status_topic').value),
            transient_qos)
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter('map_topic').value),
            self.map_callback,
            transient_qos)
        self.goal_subscription = self.create_subscription(
            PoseStamped,
            str(self.get_parameter('goal_topic').value),
            self.goal_callback,
            10)
        self.replan_subscription = self.create_subscription(
            PoseStamped,
            str(self.get_parameter('replan_topic').value),
            self.goal_callback,
            10)
        self.cancel_subscription = self.create_subscription(
            Empty,
            str(self.get_parameter('cancel_topic').value),
            self.cancel_callback,
            10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.map_lock = threading.Lock()
        self.grid_map = None
        self.plan_generation = 0
        self.publish_status('waiting_map')
        self.get_logger().info(
            'Ackermann Hybrid A* ready: '
            f'vehicle={self.planner_config.vehicle_length:.3f}x'
            f'{self.planner_config.vehicle_width:.3f}m '
            f'wheelbase={self.planner_config.wheelbase:.3f}m '
            f'grid={self.planner_config.xy_resolution:.3f}m')

    def map_callback(self, message):
        origin = message.info.origin
        grid_map = OccupancyGridMap(
            width=message.info.width,
            height=message.info.height,
            resolution=message.info.resolution,
            origin_x=origin.position.x,
            origin_y=origin.position.y,
            origin_yaw=yaw_from_quaternion(origin.orientation),
            data=message.data,
            occupied_threshold=self.occupied_threshold,
            unknown_is_occupied=self.unknown_is_occupied,
        )
        with self.map_lock:
            first_map = self.grid_map is None
            self.grid_map = grid_map
        if first_map:
            self.get_logger().info(
                f'map received: {grid_map.width}x{grid_map.height} '
                f'resolution={grid_map.resolution:.3f}m')
            self.publish_status(
                'ready',
                map_width=grid_map.width,
                map_height=grid_map.height,
                map_resolution=grid_map.resolution)

    def publish_status(self, state, **fields):
        payload = {'state': state}
        payload.update(fields)
        message = String()
        message.data = json.dumps(
            payload, ensure_ascii=True, separators=(',', ':'))
        self.status_publisher.publish(message)

    def current_pose(self):
        transform = self.tf_buffer.lookup_transform(
            self.map_frame, self.base_frame, rclpy.time.Time())
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return Pose2D(
            translation.x,
            translation.y,
            yaw_from_quaternion(rotation))

    def publish_empty_path(self):
        message = Path()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.map_frame
        self.path_publisher.publish(message)

    def cancel_callback(self, _message):
        self.plan_generation += 1
        self.publish_empty_path()
        self.publish_status('cancelled')
        self.get_logger().info('path planning cancelled')

    def goal_callback(self, message):
        with self.map_lock:
            grid_map = self.grid_map
        if grid_map is None:
            self.get_logger().error('cannot plan: fixed map is not ready')
            self.publish_empty_path()
            self.publish_status('failed', reason='map_not_ready')
            return
        if message.header.frame_id and message.header.frame_id != self.map_frame:
            self.get_logger().error(
                f'cannot plan goal in frame {message.header.frame_id}; '
                f'expected {self.map_frame}')
            self.publish_empty_path()
            self.publish_status('failed', reason='invalid_goal_frame')
            return
        try:
            start = self.current_pose()
        except TransformException as exc:
            self.get_logger().error(
                f'cannot plan: localization TF is unavailable: {exc}')
            self.publish_empty_path()
            self.publish_status('failed', reason='localization_unavailable')
            return

        goal = Pose2D(
            message.pose.position.x,
            message.pose.position.y,
            yaw_from_quaternion(message.pose.orientation))
        self.plan_generation += 1
        generation = self.plan_generation
        self.publish_empty_path()
        self.publish_status(
            'planning',
            goal={'x': goal.x, 'y': goal.y, 'yaw': goal.yaw})
        self.get_logger().info(
            f'planning start=({start.x:.3f},{start.y:.3f},'
            f'{math.degrees(start.yaw):.1f}deg) '
            f'goal=({goal.x:.3f},{goal.y:.3f},'
            f'{math.degrees(goal.yaw):.1f}deg)')
        thread = threading.Thread(
            target=self.plan_and_publish,
            args=(generation, grid_map, start, goal),
            daemon=True)
        thread.start()

    def plan_and_publish(self, generation, grid_map, start, goal):
        planner = HybridAStarPlanner(grid_map, self.planner_config)
        should_cancel = lambda: (
            generation != self.plan_generation or not rclpy.ok())
        try:
            result = planner.plan_with_yaw_fallback(
                start, goal, should_cancel=should_cancel)
        except PlanningCancelled:
            self.get_logger().info(
                'path planning cancelled by a newer goal')
            return
        except (RuntimeError, ValueError) as exc:
            if generation == self.plan_generation:
                self.get_logger().error(f'path planning failed: {exc}')
                self.publish_empty_path()
                self.publish_status('failed', reason=str(exc))
            return
        if generation != self.plan_generation:
            return

        stamp = self.get_clock().now().to_msg()
        message = Path()
        message.header.stamp = stamp
        message.header.frame_id = self.map_frame
        for pose in result.poses:
            pose_message = PoseStamped()
            pose_message.header.stamp = stamp
            pose_message.header.frame_id = self.map_frame
            pose_message.pose.position.x = pose.x
            pose_message.pose.position.y = pose.y
            qx, qy, qz, qw = yaw_quaternion(pose.yaw)
            pose_message.pose.orientation.x = qx
            pose_message.pose.orientation.y = qy
            pose_message.pose.orientation.z = qz
            pose_message.pose.orientation.w = qw
            message.poses.append(pose_message)
        yaw_mode = 'position+relaxed-yaw' if result.relaxed_goal_yaw else \
            'position+yaw'
        mode = f'approach-{yaw_mode}' if result.approach_goal else yaw_mode
        self.get_logger().info(
            f'path ready: poses={len(result.poses)} '
            f'cost={result.cost:.3f} expansions={result.expansions} '
            f'time={result.planning_time_sec:.3f}s mode={mode} '
            f'goal_distance={result.requested_goal_distance:.3f}m '
            f'goal_yaw_error='
            f'{math.degrees(result.requested_goal_yaw_error):.1f}deg '
            f'start_adjusted={result.start_adjusted} '
            f'start_adjustment={result.start_adjustment_distance:.3f}m')
        self.publish_status(
            'path_ready',
            poses=len(result.poses),
            cost=result.cost,
            expansions=result.expansions,
            planning_time_sec=result.planning_time_sec,
            mode=mode,
            approach_goal=result.approach_goal,
            relaxed_goal_yaw=result.relaxed_goal_yaw,
            final_goal_distance_m=result.requested_goal_distance,
            final_goal_yaw_error_deg=math.degrees(
                result.requested_goal_yaw_error),
            relaxed_goal_yaw_tolerance_deg=math.degrees(
                self.planner_config.relaxed_goal_yaw_tolerance),
            goal_tolerance_m=result.goal_tolerance,
            start_adjusted=result.start_adjusted,
            start_adjustment_distance_m=result.start_adjustment_distance,
            goal={'x': goal.x, 'y': goal.y, 'yaw': goal.yaw})
        self.path_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = AckermannPathPlanner()
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
