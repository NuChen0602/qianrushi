import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Empty, String
from tf2_ros import Buffer, TransformException, TransformListener

from robot_lidar_bridge.motion_control import (
    SmoothSpeedLimiter,
    curvature_steering_command,
    curvature_speed_limit,
    peak_path_curvature,
    signed_path_curvature,
)


def clamp(value, low, high):
    return max(low, min(high, value))


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class GoalNavigator(Node):
    """Tracks the Hybrid A* path with a smooth heading PID."""

    def __init__(self):
        super().__init__('goal_navigator')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('path_topic', '/planned_path')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('replan_topic', '/navigation/replan')
        self.declare_parameter('planner_status_topic', '/planner/status')
        self.declare_parameter(
            'localization_ok_topic', '/navigation/localization_ok')
        self.declare_parameter(
            'localization_status_topic', '/navigation/localization_status')
        self.declare_parameter(
            'path_blocked_topic', '/navigation/path_blocked')
        self.declare_parameter(
            'obstacle_status_topic', '/navigation/obstacle_status')
        self.declare_parameter('cancel_topic', '/navigation/cancel')
        self.declare_parameter(
            'emergency_stop_topic', '/navigation/emergency_stop')
        self.declare_parameter('status_topic', '/navigation/status')
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('goal_tolerance_m', 0.10)
        self.declare_parameter('goal_yaw_tolerance_deg', 10.0)
        self.declare_parameter('final_yaw_control_radius_m', 0.18)
        self.declare_parameter('max_speed_mps', 0.18)
        self.declare_parameter('min_speed_mps', 0.08)
        self.declare_parameter('slow_radius_m', 0.45)
        self.declare_parameter('lookahead_min_m', 0.12)
        self.declare_parameter('lookahead_max_m', 0.28)
        self.declare_parameter('lookahead_speed_gain', 0.65)
        self.declare_parameter('lookahead_curvature_gain', 0.30)
        self.declare_parameter('path_index_search_forward', 8)
        self.declare_parameter('max_cross_track_error_m', 0.30)
        self.declare_parameter('cross_track_kp', 1.20)
        self.declare_parameter('cross_track_steering_limit', 0.25)
        self.declare_parameter('heading_pid_kp', 0.75)
        self.declare_parameter('heading_pid_ki', 0.0)
        self.declare_parameter('heading_pid_kd', 0.10)
        self.declare_parameter('heading_pid_integral_limit', 0.35)
        self.declare_parameter('heading_derivative_filter', 0.25)
        self.declare_parameter('max_steering_command', 0.85)
        self.declare_parameter('left_steering_gain', 1.0)
        self.declare_parameter('right_steering_gain', 1.0)
        self.declare_parameter('max_steering_rate_per_sec', 1.2)
        self.declare_parameter('max_front_wheel_angle_deg', 37.0)
        self.declare_parameter('wheelbase_m', 0.18)
        self.declare_parameter('curvature_feedforward_gain', 0.85)
        self.declare_parameter('curvature_feedforward_lookahead_m', 0.24)
        self.declare_parameter('max_steering_rate_deg_per_sec', 45.0)
        self.declare_parameter('heading_slow_angle_rad', 0.45)
        self.declare_parameter('curvature_lookahead_m', 0.45)
        self.declare_parameter('curvature_speed_gain', 0.45)
        self.declare_parameter('min_curve_speed_mps', 0.07)
        self.declare_parameter('max_acceleration_mps2', 0.25)
        self.declare_parameter('max_deceleration_mps2', 0.35)
        self.declare_parameter('max_jerk_mps3', 1.2)
        self.declare_parameter('path_timeout_sec', 120.0)
        self.declare_parameter('status_period_sec', 0.5)
        self.declare_parameter('replan_cooldown_sec', 2.0)
        self.declare_parameter('max_recovery_attempts', 3)
        self.declare_parameter('recovery_retry_delay_sec', 1.5)
        self.declare_parameter('obstacle_replan_delay_sec', 0.8)
        self.declare_parameter('obstacle_failure_timeout_sec', 20.0)
        self.declare_parameter('localization_failure_timeout_sec', 15.0)
        self.declare_parameter('safety_status_timeout_sec', 1.5)
        self.declare_parameter('stuck_timeout_sec', 3.0)
        self.declare_parameter('minimum_progress_m', 0.03)
        self.declare_parameter('degraded_localization_speed_scale', 0.65)

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.goal_tolerance_m = max(
            float(self.get_parameter('goal_tolerance_m').value), 0.02)
        self.goal_yaw_tolerance_rad = math.radians(max(
            float(self.get_parameter('goal_yaw_tolerance_deg').value), 1.0))
        self.final_yaw_control_radius_m = max(
            float(self.get_parameter('final_yaw_control_radius_m').value),
            self.goal_tolerance_m)
        self.max_speed_mps = max(
            float(self.get_parameter('max_speed_mps').value), 0.01)
        self.min_speed_mps = clamp(
            float(self.get_parameter('min_speed_mps').value),
            0.0,
            self.max_speed_mps)
        self.slow_radius_m = max(
            float(self.get_parameter('slow_radius_m').value),
            self.goal_tolerance_m + 0.01)
        self.lookahead_min_m = max(
            float(self.get_parameter('lookahead_min_m').value), 0.04)
        self.lookahead_max_m = max(
            float(self.get_parameter('lookahead_max_m').value),
            self.lookahead_min_m)
        self.lookahead_speed_gain = max(
            float(self.get_parameter('lookahead_speed_gain').value), 0.0)
        self.lookahead_curvature_gain = max(
            float(self.get_parameter('lookahead_curvature_gain').value), 0.0)
        self.path_index_search_forward = max(
            int(self.get_parameter('path_index_search_forward').value), 1)
        self.max_cross_track_error_m = max(
            float(self.get_parameter('max_cross_track_error_m').value), 0.05)
        self.cross_track_kp = max(
            float(self.get_parameter('cross_track_kp').value), 0.0)
        self.cross_track_steering_limit = clamp(
            abs(float(self.get_parameter(
                'cross_track_steering_limit').value)),
            0.0,
            1.0)
        self.heading_pid_kp = float(
            self.get_parameter('heading_pid_kp').value)
        self.heading_pid_ki = float(
            self.get_parameter('heading_pid_ki').value)
        self.heading_pid_kd = float(
            self.get_parameter('heading_pid_kd').value)
        self.heading_pid_integral_limit = max(
            abs(float(
                self.get_parameter('heading_pid_integral_limit').value)),
            0.0)
        self.heading_derivative_filter = clamp(
            float(self.get_parameter('heading_derivative_filter').value),
            0.0,
            1.0)
        self.max_steering_command = clamp(
            abs(float(self.get_parameter('max_steering_command').value)),
            0.05,
            1.0)
        self.left_steering_gain = clamp(
            float(self.get_parameter('left_steering_gain').value),
            0.2,
            2.0)
        self.right_steering_gain = clamp(
            float(self.get_parameter('right_steering_gain').value),
            0.2,
            2.0)
        legacy_steering_rate = max(
            abs(float(self.get_parameter('max_steering_rate_per_sec').value)),
            0.05)
        self.max_front_wheel_angle_deg = max(
            abs(float(self.get_parameter('max_front_wheel_angle_deg').value)),
            1.0)
        self.wheelbase_m = max(
            float(self.get_parameter('wheelbase_m').value), 0.01)
        self.curvature_feedforward_gain = max(
            float(self.get_parameter('curvature_feedforward_gain').value),
            0.0)
        self.curvature_feedforward_lookahead_m = max(
            float(self.get_parameter(
                'curvature_feedforward_lookahead_m').value), 0.04)
        steering_rate_deg = float(
            self.get_parameter('max_steering_rate_deg_per_sec').value)
        self.max_steering_rate_per_sec = (
            abs(steering_rate_deg) / self.max_front_wheel_angle_deg
            if abs(steering_rate_deg) > 1e-6 else legacy_steering_rate)
        self.heading_slow_angle_rad = max(
            float(self.get_parameter('heading_slow_angle_rad').value), 0.01)
        self.curvature_lookahead_m = max(
            float(self.get_parameter('curvature_lookahead_m').value), 0.10)
        self.curvature_speed_gain = max(
            float(self.get_parameter('curvature_speed_gain').value), 0.0)
        self.min_curve_speed_mps = clamp(
            float(self.get_parameter('min_curve_speed_mps').value),
            0.01,
            self.max_speed_mps)
        self.max_acceleration_mps2 = max(
            float(self.get_parameter('max_acceleration_mps2').value), 0.01)
        self.max_deceleration_mps2 = max(
            float(self.get_parameter('max_deceleration_mps2').value), 0.01)
        self.max_jerk_mps3 = max(
            float(self.get_parameter('max_jerk_mps3').value), 0.01)
        self.path_timeout_sec = max(
            float(self.get_parameter('path_timeout_sec').value), 1.0)
        self.status_period_sec = max(
            float(self.get_parameter('status_period_sec').value), 0.1)
        self.replan_cooldown_sec = max(
            float(self.get_parameter('replan_cooldown_sec').value), 0.5)
        self.max_recovery_attempts = max(
            int(self.get_parameter('max_recovery_attempts').value), 0)
        self.recovery_retry_delay_sec = max(
            float(self.get_parameter('recovery_retry_delay_sec').value), 0.1)
        self.obstacle_replan_delay_sec = max(
            float(self.get_parameter('obstacle_replan_delay_sec').value), 0.1)
        self.obstacle_failure_timeout_sec = max(
            float(self.get_parameter('obstacle_failure_timeout_sec').value),
            self.obstacle_replan_delay_sec)
        self.localization_failure_timeout_sec = max(
            float(self.get_parameter(
                'localization_failure_timeout_sec').value), 1.0)
        self.safety_status_timeout_sec = max(
            float(self.get_parameter('safety_status_timeout_sec').value), 0.5)
        self.stuck_timeout_sec = max(
            float(self.get_parameter('stuck_timeout_sec').value), 1.0)
        self.minimum_progress_m = max(
            float(self.get_parameter('minimum_progress_m').value), 0.005)
        self.degraded_localization_speed_scale = clamp(
            float(self.get_parameter(
                'degraded_localization_speed_scale').value), 0.2, 1.0)

        path_topic = str(self.get_parameter('path_topic').value)
        cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        rate = max(float(self.get_parameter('control_rate_hz').value), 1.0)
        self.control_period_sec = 1.0 / rate
        transient_qos = QoSProfile(depth=1)
        transient_qos.reliability = ReliabilityPolicy.RELIABLE
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.replan_pub = self.create_publisher(
            PoseStamped,
            str(self.get_parameter('replan_topic').value),
            10)
        self.status_pub = self.create_publisher(
            String,
            str(self.get_parameter('status_topic').value),
            transient_qos)
        self.path_sub = self.create_subscription(
            Path, path_topic, self.path_callback, transient_qos)
        self.goal_sub = self.create_subscription(
            PoseStamped,
            str(self.get_parameter('goal_topic').value),
            self.goal_callback,
            10)
        self.planner_status_sub = self.create_subscription(
            String,
            str(self.get_parameter('planner_status_topic').value),
            self.planner_status_callback,
            transient_qos)
        self.localization_ok_sub = self.create_subscription(
            Bool,
            str(self.get_parameter('localization_ok_topic').value),
            self.localization_ok_callback,
            transient_qos)
        self.localization_status_sub = self.create_subscription(
            String,
            str(self.get_parameter('localization_status_topic').value),
            self.localization_status_callback,
            transient_qos)
        self.path_blocked_sub = self.create_subscription(
            Bool,
            str(self.get_parameter('path_blocked_topic').value),
            self.path_blocked_callback,
            transient_qos)
        self.obstacle_status_sub = self.create_subscription(
            String,
            str(self.get_parameter('obstacle_status_topic').value),
            self.obstacle_status_callback,
            transient_qos)
        self.timer = self.create_timer(1.0 / rate, self.control_loop)
        self.cancel_sub = self.create_subscription(
            Empty,
            str(self.get_parameter('cancel_topic').value),
            self.cancel_callback,
            10)
        self.emergency_stop_sub = self.create_subscription(
            Bool,
            str(self.get_parameter('emergency_stop_topic').value),
            self.emergency_stop_callback,
            10)

        self.path = []
        self.path_index = 0
        self.current_path_mode = ''
        self.current_path_approach_goal = False
        self.current_path_relaxed_goal_yaw = False
        self.current_path_final_goal_distance_m = None
        self.current_path_goal_tolerance_m = None
        self.path_received_time = 0.0
        self.last_status_time = 0.0
        self.last_stop_publish_time = 0.0
        self.heading_integral = 0.0
        self.last_heading_error = 0.0
        self.heading_derivative = 0.0
        self.last_control_time = 0.0
        self.steering_command = 0.0
        self.motion_direction = 0
        self.emergency_stopped = False
        self.active_goal = None
        self.last_replan_time = 0.0
        self.last_status_state = ''
        self.planner_status = {'state': 'starting'}
        self.localization_status = {'state': 'initializing'}
        self.localization_ok = False
        self.last_localization_signal_time = 0.0
        self.localization_bad_since = 0.0
        self.path_blocked = False
        self.obstacle_status = {'state': 'clear'}
        self.last_obstacle_signal_time = 0.0
        self.obstacle_blocked_since = 0.0
        self.recovery_attempts = 0
        self.recovery_reason = ''
        self.next_recovery_time = 0.0
        self.awaiting_replan = False
        self.last_progress_pose = None
        self.last_progress_remaining = math.inf
        self.last_progress_time = 0.0
        self.speed_limiter = SmoothSpeedLimiter(
            self.max_acceleration_mps2,
            self.max_deceleration_mps2,
            self.max_jerk_mps3)
        self.last_speed_update_time = 0.0

        self.get_logger().info(
            f'path follower ready: path={path_topic} cmd={cmd_vel_topic} '
            f'frames={self.map_frame}->{self.base_frame} '
            f'heading_pid=({self.heading_pid_kp:.3f},'
            f'{self.heading_pid_ki:.3f},{self.heading_pid_kd:.3f}) '
            f'accel/decel={self.max_acceleration_mps2:.2f}/'
            f'{self.max_deceleration_mps2:.2f}m/s2 '
            f'steering_rate={steering_rate_deg:.1f}deg/s')
        self.publish_status('idle')

    def publish_status(self, state, **fields):
        payload = {'state': state}
        payload.update(fields)
        message = String()
        message.data = json.dumps(
            payload, ensure_ascii=True, separators=(',', ':'))
        self.status_pub.publish(message)
        self.last_status_state = state

    @staticmethod
    def decode_status(message):
        try:
            payload = json.loads(message.data)
            return payload if isinstance(payload, dict) else {
                'state': str(payload)}
        except json.JSONDecodeError:
            return {'state': message.data}

    def reset_recovery(self):
        self.recovery_attempts = 0
        self.recovery_reason = ''
        self.next_recovery_time = 0.0
        self.awaiting_replan = False
        self.localization_bad_since = 0.0
        self.obstacle_blocked_since = 0.0
        self.last_progress_pose = None
        self.last_progress_remaining = math.inf
        self.last_progress_time = 0.0

    def goal_callback(self, message):
        if (message.header.frame_id and
                message.header.frame_id != self.map_frame):
            return
        self.active_goal = (
            float(message.pose.position.x),
            float(message.pose.position.y),
            yaw_from_quaternion(message.pose.orientation),
        )
        self.current_path_mode = ''
        self.current_path_approach_goal = False
        self.current_path_relaxed_goal_yaw = False
        self.current_path_final_goal_distance_m = None
        self.current_path_goal_tolerance_m = None
        self.reset_recovery()
        self.publish_status(
            'waiting_for_path',
            goal={
                'x': self.active_goal[0],
                'y': self.active_goal[1],
                'yaw': self.active_goal[2],
            },
            recovery_attempt=0,
            recovery_limit=self.max_recovery_attempts)

    def planner_status_callback(self, message):
        self.planner_status = self.decode_status(message)
        state = self.planner_status.get('state')
        if state == 'path_ready':
            self.current_path_mode = str(
                self.planner_status.get('mode', ''))
            self.current_path_approach_goal = bool(
                self.planner_status.get('approach_goal', False))
            self.current_path_relaxed_goal_yaw = bool(
                self.planner_status.get('relaxed_goal_yaw', False))
            self.current_path_final_goal_distance_m = (
                self.planner_status.get('final_goal_distance_m'))
            try:
                self.current_path_goal_tolerance_m = float(
                    self.planner_status.get('goal_tolerance_m'))
            except (TypeError, ValueError):
                self.current_path_goal_tolerance_m = None
            self.awaiting_replan = False
            self.next_recovery_time = 0.0
            return
        if state != 'failed' or self.active_goal is None:
            return
        self.awaiting_replan = False
        reason = str(self.planner_status.get('reason', 'planning_failed'))
        if self.recovery_attempts >= self.max_recovery_attempts:
            self.fail_navigation(f'planning_failed:{reason}')
            return
        self.schedule_recovery(
            f'planning_failed:{reason}', self.recovery_retry_delay_sec)

    def localization_ok_callback(self, message):
        self.localization_ok = bool(message.data)
        self.last_localization_signal_time = time.monotonic()

    def localization_status_callback(self, message):
        self.localization_status = self.decode_status(message)
        self.last_localization_signal_time = time.monotonic()

    def path_blocked_callback(self, message):
        self.path_blocked = bool(message.data)
        self.last_obstacle_signal_time = time.monotonic()

    def obstacle_status_callback(self, message):
        self.obstacle_status = self.decode_status(message)
        self.last_obstacle_signal_time = time.monotonic()

    def cancel_callback(self, _message):
        self.active_goal = None
        self.reset_recovery()
        self.clear_path()
        self.publish_status('cancelled')
        self.get_logger().info('navigation cancelled')

    def emergency_stop_callback(self, message):
        self.emergency_stopped = bool(message.data)
        self.active_goal = None
        self.reset_recovery()
        self.clear_path()
        if self.emergency_stopped:
            self.publish_status('emergency_stopped')
            self.get_logger().warning('navigation emergency stop engaged')
        else:
            self.publish_status('idle')
            self.get_logger().info('navigation emergency stop released')

    def reset_heading_pid(self):
        self.heading_integral = 0.0
        self.last_heading_error = 0.0
        self.heading_derivative = 0.0
        self.last_control_time = 0.0
        self.steering_command = 0.0

    def path_callback(self, message):
        if (message.header.frame_id and
                message.header.frame_id != self.map_frame):
            self.get_logger().error(
                f'ignored path in frame {message.header.frame_id}; '
                f'expected {self.map_frame}')
            return
        if not message.poses:
            if self.path:
                self.get_logger().info('path cleared; stopping')
            self.path = []
            self.path_index = 0
            self.current_path_mode = ''
            self.current_path_approach_goal = False
            self.current_path_relaxed_goal_yaw = False
            self.current_path_final_goal_distance_m = None
            self.current_path_goal_tolerance_m = None
            self.reset_heading_pid()
            self.publish_stop()
            if (not self.emergency_stopped and
                    self.last_status_state not in (
                        'cancelled', 'emergency_stopped', 'failed')):
                state = 'replanning' if self.awaiting_replan else \
                    'waiting_for_path'
                self.publish_status(
                    state,
                    reason=self.recovery_reason or None,
                    recovery_attempt=self.recovery_attempts,
                    recovery_limit=self.max_recovery_attempts)
            return
        self.path = [
            (
                pose.pose.position.x,
                pose.pose.position.y,
                yaw_from_quaternion(pose.pose.orientation),
            )
            for pose in message.poses
        ]
        self.path_index = 0
        self.path_received_time = time.monotonic()
        self.motion_direction = 0
        goal_x, goal_y, goal_yaw = self.path[-1]
        if self.active_goal is None:
            self.active_goal = (goal_x, goal_y, goal_yaw)
        self.awaiting_replan = False
        self.next_recovery_time = 0.0
        self.last_progress_pose = None
        self.last_progress_remaining = math.inf
        self.last_progress_time = time.monotonic()
        self.reset_heading_pid()
        self.get_logger().info(
            f'new planned path received: poses={len(self.path)} '
            f'goal=({self.path[-1][0]:.3f},{self.path[-1][1]:.3f}) '
            f'mode={self.current_path_mode or "unknown"}')
        self.publish_status(
            'following',
            path_poses=len(self.path),
            path_mode=self.current_path_mode or None,
            approach_goal=self.current_path_approach_goal,
            goal={
                'x': self.active_goal[0],
                'y': self.active_goal[1],
                'yaw': self.active_goal[2],
            },
            recovery_attempt=self.recovery_attempts,
            recovery_limit=self.max_recovery_attempts)

    def publish_stop(self):
        now = time.monotonic()
        if now - self.last_stop_publish_time > 0.05:
            self.cmd_pub.publish(Twist())
            self.last_stop_publish_time = now
        self.steering_command = 0.0
        self.speed_limiter.reset()
        self.last_speed_update_time = 0.0

    def update_speed_command(self, target_speed, now):
        dt = self.control_period_sec
        if self.last_speed_update_time > 0.0:
            dt = clamp(
                now - self.last_speed_update_time,
                0.001,
                0.2)
        self.last_speed_update_time = now
        return self.speed_limiter.update(target_speed, dt)

    def current_pose(self):
        transform = self.tf_buffer.lookup_transform(
            self.map_frame, self.base_frame, rclpy.time.Time())
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return translation.x, translation.y, yaw_from_quaternion(rotation)

    def update_heading_pid(self, error, now):
        if self.last_control_time <= 0.0:
            self.last_control_time = now
            self.last_heading_error = error
            return 0.0

        dt = clamp(now - self.last_control_time, 0.001, 0.2)
        raw_derivative = normalize_angle(error - self.last_heading_error) / dt
        self.heading_derivative = (
            self.heading_derivative_filter * raw_derivative +
            (1.0 - self.heading_derivative_filter) * self.heading_derivative)

        self.heading_integral += error * dt
        if self.heading_pid_integral_limit > 0.0:
            self.heading_integral = clamp(
                self.heading_integral,
                -self.heading_pid_integral_limit,
                self.heading_pid_integral_limit)
        else:
            self.heading_integral = 0.0

        target_steering = (
            self.heading_pid_kp * error +
            self.heading_pid_ki * self.heading_integral +
            self.heading_pid_kd * self.heading_derivative)
        target_steering = clamp(
            target_steering,
            -self.max_steering_command,
            self.max_steering_command)

        max_delta = self.max_steering_rate_per_sec * dt
        self.steering_command += clamp(
            target_steering - self.steering_command,
            -max_delta,
            max_delta)
        self.steering_command = clamp(
            self.steering_command,
            -self.max_steering_command,
            self.max_steering_command)

        self.last_heading_error = error
        self.last_control_time = now
        return self.steering_command

    def update_nearest_path_index(self, x, y):
        begin = max(0, self.path_index - 3)
        end = min(
            len(self.path),
            self.path_index + self.path_index_search_forward + 1)
        best_index = begin
        best_distance = math.inf
        for index in range(begin, end):
            px, py, _ = self.path[index]
            distance = math.hypot(px - x, py - y)
            if distance < best_distance:
                best_distance = distance
                best_index = index
        self.path_index = max(self.path_index, best_index)
        return best_distance

    def signed_cross_track_error(self, x, y):
        """Return lateral error in meters; positive means robot is left of path."""
        if len(self.path) < 2:
            return 0.0
        index = min(max(self.path_index, 0), len(self.path) - 2)
        ax, ay, _ = self.path[index]
        bx, by, _ = self.path[index + 1]
        dx = bx - ax
        dy = by - ay
        length = math.hypot(dx, dy)
        if length <= 1e-6 and index > 0:
            ax, ay, _ = self.path[index - 1]
            bx, by, _ = self.path[index]
            dx = bx - ax
            dy = by - ay
            length = math.hypot(dx, dy)
        if length <= 1e-6:
            return 0.0
        return (dx * (y - ay) - dy * (x - ax)) / length

    def cross_track_steering_correction(self, signed_error, direction):
        if self.cross_track_kp <= 0.0:
            return 0.0
        correction = -float(direction) * self.cross_track_kp * signed_error
        return clamp(
            correction,
            -self.cross_track_steering_limit,
            self.cross_track_steering_limit)

    def apply_directional_steering_gain(self, steering):
        if steering > 0.0:
            return steering * self.left_steering_gain
        if steering < 0.0:
            return steering * self.right_steering_gain
        return 0.0

    def path_remaining_distance(self, x, y):
        if not self.path:
            return 0.0
        px, py, _ = self.path[self.path_index]
        remaining = math.hypot(px - x, py - y)
        for index in range(self.path_index, len(self.path) - 1):
            ax, ay, _ = self.path[index]
            bx, by, _ = self.path[index + 1]
            remaining += math.hypot(bx - ax, by - ay)
        return remaining

    def lookahead_target(self, x, y, lookahead, direction):
        index = self.path_index
        previous_x = x
        previous_y = y
        accumulated = 0.0
        while index < len(self.path):
            target_x, target_y, target_yaw = self.path[index]
            accumulated += math.hypot(
                target_x - previous_x, target_y - previous_y)
            if accumulated >= lookahead or index == len(self.path) - 1:
                return index, target_x, target_y, target_yaw
            if (index > self.path_index and
                    self.path_motion_direction(index) != direction):
                return index, target_x, target_y, target_yaw
            previous_x = target_x
            previous_y = target_y
            index += 1
        return len(self.path) - 1, *self.path[-1]

    def path_motion_direction(self, index):
        if len(self.path) < 2:
            return 1
        index = min(index, len(self.path) - 2)
        ax, ay, yaw = self.path[index]
        bx, by, _ = self.path[index + 1]
        projection = (
            (bx - ax) * math.cos(yaw) +
            (by - ay) * math.sin(yaw))
        return 1 if projection >= 0.0 else -1

    def clear_path(self):
        self.path = []
        self.path_index = 0
        self.motion_direction = 0
        self.current_path_mode = ''
        self.current_path_approach_goal = False
        self.current_path_relaxed_goal_yaw = False
        self.current_path_final_goal_distance_m = None
        self.current_path_goal_tolerance_m = None
        self.reset_heading_pid()
        self.publish_stop()

    def fail_navigation(self, reason, **extra_fields):
        failed_goal = self.active_goal
        attempts = self.recovery_attempts
        self.active_goal = None
        self.next_recovery_time = 0.0
        self.awaiting_replan = False
        self.clear_path()
        fields = {
            'reason': reason,
            'recovery_attempts': attempts,
            'recovery_limit': self.max_recovery_attempts,
        }
        fields.update(extra_fields)
        if failed_goal is not None:
            fields['goal'] = {
                'x': failed_goal[0],
                'y': failed_goal[1],
                'yaw': failed_goal[2],
            }
        self.publish_status('failed', **fields)
        self.get_logger().error(
            f'navigation failed: {reason}; recovery attempts={attempts}')

    def schedule_recovery(self, reason, delay=None):
        if self.active_goal is None:
            return False
        if self.recovery_attempts >= self.max_recovery_attempts:
            self.fail_navigation(reason)
            return False
        wait = self.recovery_retry_delay_sec if delay is None else max(
            float(delay), 0.0)
        self.recovery_reason = str(reason)
        due = time.monotonic() + wait
        if self.next_recovery_time <= 0.0 or due < self.next_recovery_time:
            self.next_recovery_time = due
        self.publish_status(
            'recovering',
            reason=self.recovery_reason,
            retry_in_sec=wait,
            recovery_attempt=self.recovery_attempts,
            recovery_limit=self.max_recovery_attempts)
        return True

    def request_replan(self, reason='replan_requested'):
        if self.active_goal is None:
            return False
        now = time.monotonic()
        if now - self.last_replan_time < self.replan_cooldown_sec:
            remaining = self.replan_cooldown_sec - (
                now - self.last_replan_time)
            self.schedule_recovery(reason, remaining)
            return False
        if self.recovery_attempts >= self.max_recovery_attempts:
            self.fail_navigation(reason)
            return False
        goal_x, goal_y, goal_yaw = self.active_goal
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.map_frame
        message.pose.position.x = goal_x
        message.pose.position.y = goal_y
        message.pose.orientation.z = math.sin(0.5 * goal_yaw)
        message.pose.orientation.w = math.cos(0.5 * goal_yaw)
        self.last_replan_time = now
        self.recovery_attempts += 1
        self.recovery_reason = str(reason)
        self.next_recovery_time = 0.0
        self.awaiting_replan = True
        self.replan_pub.publish(message)
        self.publish_status(
            'replanning',
            reason=self.recovery_reason,
            recovery_attempt=self.recovery_attempts,
            recovery_limit=self.max_recovery_attempts,
            goal={'x': goal_x, 'y': goal_y, 'yaw': goal_yaw})
        self.get_logger().warning(
            f'requesting path recovery {self.recovery_attempts}/'
            f'{self.max_recovery_attempts}: {self.recovery_reason}')
        return True

    def update_progress_watchdog(self, x, y, remaining, now):
        if self.last_progress_pose is None:
            self.last_progress_pose = (x, y)
            self.last_progress_remaining = remaining
            self.last_progress_time = now
            return False
        moved = math.hypot(
            x - self.last_progress_pose[0],
            y - self.last_progress_pose[1])
        advanced = self.last_progress_remaining - remaining
        if (moved >= self.minimum_progress_m or
                advanced >= self.minimum_progress_m):
            self.last_progress_pose = (x, y)
            self.last_progress_remaining = remaining
            self.last_progress_time = now
            return False
        return now - self.last_progress_time >= self.stuck_timeout_sec

    def control_loop(self):
        if self.emergency_stopped:
            self.publish_stop()
            return
        now = time.monotonic()

        localization_fresh = (
            now - self.last_localization_signal_time <=
            self.safety_status_timeout_sec)
        localization_ready = self.localization_ok and localization_fresh
        if self.active_goal is not None and not localization_ready:
            self.publish_stop()
            if self.localization_bad_since <= 0.0:
                self.localization_bad_since = now
            elapsed = now - self.localization_bad_since
            reason = self.localization_status.get('reason') or (
                'safety_monitor_timeout' if not localization_fresh else
                'localization_not_ready')
            if elapsed >= self.localization_failure_timeout_sec:
                self.fail_navigation(f'localization_lost:{reason}')
                return
            if now - self.last_status_time >= self.status_period_sec:
                self.last_status_time = now
                self.publish_status(
                    'localization_paused',
                    reason=reason,
                    paused_sec=elapsed,
                    localization=self.localization_status,
                    recovery_attempt=self.recovery_attempts,
                    recovery_limit=self.max_recovery_attempts)
            return

        if self.localization_bad_since > 0.0:
            paused_for = now - self.localization_bad_since
            self.localization_bad_since = 0.0
            if self.active_goal is not None and paused_for > 0.2:
                self.clear_path()
                self.schedule_recovery('localization_recovered', 0.0)

        if self.next_recovery_time > 0.0:
            self.publish_stop()
            if now >= self.next_recovery_time:
                reason = self.recovery_reason or 'recovery_retry'
                self.next_recovery_time = 0.0
                self.clear_path()
                self.request_replan(reason)
            return

        if self.awaiting_replan:
            self.publish_stop()
            return

        if not self.path:
            self.publish_stop()
            return

        obstacle_fresh = (
            now - self.last_obstacle_signal_time <=
            self.safety_status_timeout_sec)
        if not obstacle_fresh:
            self.publish_stop()
            self.clear_path()
            self.schedule_recovery('obstacle_monitor_timeout', 0.0)
            return

        if self.path_blocked:
            self.publish_stop()
            if self.obstacle_blocked_since <= 0.0:
                self.obstacle_blocked_since = now
            blocked_for = now - self.obstacle_blocked_since
            if blocked_for >= self.obstacle_failure_timeout_sec:
                if self.recovery_attempts >= self.max_recovery_attempts:
                    self.fail_navigation('dynamic_obstacle_persisted')
                    return
            if (blocked_for >= self.obstacle_replan_delay_sec and
                    self.next_recovery_time <= 0.0 and
                    not self.awaiting_replan):
                self.schedule_recovery('dynamic_obstacle', 0.0)
            if now - self.last_status_time >= self.status_period_sec:
                self.last_status_time = now
                self.publish_status(
                    'obstacle_waiting',
                    blocked_sec=blocked_for,
                    obstacle=self.obstacle_status,
                    recovery_attempt=self.recovery_attempts,
                    recovery_limit=self.max_recovery_attempts)
            return
        self.obstacle_blocked_since = 0.0

        if now - self.path_received_time > self.path_timeout_sec:
            self.clear_path()
            self.schedule_recovery('path_timeout', 0.0)
            return

        try:
            x, y, yaw = self.current_pose()
        except TransformException as exc:
            self.publish_stop()
            if self.localization_bad_since <= 0.0:
                self.localization_bad_since = now
            if now - self.last_status_time >= self.status_period_sec:
                self.last_status_time = now
                self.get_logger().warning(
                    f'waiting for localization TF: {exc}')
                self.publish_status(
                    'localization_paused', reason='tf_unavailable')
            return

        cross_track = self.update_nearest_path_index(x, y)
        signed_cross_track = self.signed_cross_track_error(x, y)
        if cross_track > self.max_cross_track_error_m:
            self.get_logger().error(
                f'path tracking error {cross_track:.3f}m exceeds '
                f'{self.max_cross_track_error_m:.3f}m; replanning required')
            self.clear_path()
            self.schedule_recovery('cross_track_error', 0.0)
            return

        goal_x, goal_y, path_goal_yaw = self.path[-1]
        requested_goal = self.active_goal or (goal_x, goal_y, path_goal_yaw)
        requested_goal_yaw = requested_goal[2]
        final_goal_yaw = (
            path_goal_yaw if self.current_path_relaxed_goal_yaw
            else requested_goal_yaw)
        path_goal_distance = math.hypot(goal_x - x, goal_y - y)
        requested_goal_distance = math.hypot(
            requested_goal[0] - x, requested_goal[1] - y)
        goal_yaw_error = normalize_angle(final_goal_yaw - yaw)
        requested_goal_yaw_error = normalize_angle(requested_goal_yaw - yaw)
        approach_tolerance_m = (
            self.current_path_goal_tolerance_m
            if self.current_path_goal_tolerance_m is not None
            else self.goal_tolerance_m)
        approach_tolerance_m = max(
            float(approach_tolerance_m), self.goal_tolerance_m)
        goal_position_reached = (
            requested_goal_distance <= approach_tolerance_m
            if self.current_path_approach_goal
            else path_goal_distance <= self.goal_tolerance_m)
        goal_yaw_reached = (
            abs(goal_yaw_error) <= self.goal_yaw_tolerance_rad)
        requested_goal_yaw_reached = (
            abs(requested_goal_yaw_error) <= self.goal_yaw_tolerance_rad)
        if (goal_position_reached and self.current_path_relaxed_goal_yaw and
                not requested_goal_yaw_reached):
            path_mode = self.current_path_mode
            fields = {
                'path_goal_distance_m': path_goal_distance,
                'goal_distance_m': requested_goal_distance,
                'yaw_error_deg': math.degrees(goal_yaw_error),
                'requested_goal_yaw_error_deg': math.degrees(
                    requested_goal_yaw_error),
                'goal_yaw_tolerance_deg': math.degrees(
                    self.goal_yaw_tolerance_rad),
                'path_mode': path_mode or None,
                'approach_goal': self.current_path_approach_goal,
                'relaxed_goal_yaw': self.current_path_relaxed_goal_yaw,
                'recovery_attempts': self.recovery_attempts,
            }
            if self.recovery_attempts >= self.max_recovery_attempts:
                self.get_logger().error(
                    'goal position was approached but requested yaw remained '
                    f'off by '
                    f'{math.degrees(requested_goal_yaw_error):.1f}deg '
                    'after yaw-constrained replans; stopping as failed')
                self.fail_navigation('goal_yaw_unreachable', **fields)
                return
            self.get_logger().warning(
                'path reached with relaxed goal yaw but requested yaw is '
                f'still off by '
                f'{math.degrees(requested_goal_yaw_error):.1f}deg; '
                'requesting a yaw-constrained replan')
            self.publish_status('aligning_goal_yaw', **fields)
            self.clear_path()
            self.request_replan('goal_yaw_misaligned')
            return
        remaining = self.path_remaining_distance(x, y)
        terminal_remaining_m = min(0.02, self.goal_tolerance_m * 0.25)
        path_end_reached = (
            (self.path_index >= len(self.path) - 1 or
             remaining <= terminal_remaining_m) and
            path_goal_distance <= self.final_yaw_control_radius_m)
        if path_end_reached and not goal_yaw_reached:
            path_mode = self.current_path_mode
            fields = {
                'path_goal_distance_m': path_goal_distance,
                'goal_distance_m': requested_goal_distance,
                'path_remaining_m': remaining,
                'terminal_remaining_m': terminal_remaining_m,
                'yaw_error_deg': math.degrees(goal_yaw_error),
                'requested_goal_yaw_error_deg': math.degrees(
                    requested_goal_yaw_error),
                'goal_yaw_tolerance_deg': math.degrees(
                    self.goal_yaw_tolerance_rad),
                'goal_tolerance_m': approach_tolerance_m,
                'path_mode': path_mode or None,
                'approach_goal': self.current_path_approach_goal,
                'relaxed_goal_yaw': self.current_path_relaxed_goal_yaw,
                'recovery_attempts': self.recovery_attempts,
            }
            if self.recovery_attempts >= self.max_recovery_attempts:
                self.get_logger().error(
                    'path endpoint reached but requested yaw remained '
                    f'off by {math.degrees(requested_goal_yaw_error):.1f}deg; '
                    'stopping as failed')
                self.fail_navigation('goal_yaw_unreachable', **fields)
                return
            self.get_logger().warning(
                'path endpoint reached but goal yaw is still off by '
                f'{math.degrees(requested_goal_yaw_error):.1f}deg; '
                'requesting a yaw-constrained replan')
            self.publish_status('aligning_goal_yaw', **fields)
            self.clear_path()
            self.request_replan('goal_yaw_misaligned')
            return
        if goal_position_reached and goal_yaw_reached:
            reached_state = (
                'approached'
                if (self.current_path_approach_goal and
                    requested_goal_distance > self.goal_tolerance_m)
                else 'reached')
            self.get_logger().info(
                f'goal {reached_state} path_distance='
                f'{path_goal_distance:.3f}m requested_distance='
                f'{requested_goal_distance:.3f}m '
                f'yaw_error={math.degrees(goal_yaw_error):.1f}deg; stopping')
            reached_goal = self.active_goal
            path_mode = self.current_path_mode
            approach_goal = self.current_path_approach_goal
            relaxed_goal_yaw = self.current_path_relaxed_goal_yaw
            self.active_goal = None
            self.clear_path()
            fields = {
                'distance': requested_goal_distance,
                'path_goal_distance': path_goal_distance,
                'yaw_error_deg': math.degrees(goal_yaw_error),
                'requested_yaw_error_deg': math.degrees(
                    requested_goal_yaw_error),
                'path_mode': path_mode or None,
                'approach_goal': approach_goal,
                'relaxed_goal_yaw': relaxed_goal_yaw,
                'recovery_attempts': self.recovery_attempts,
            }
            if reached_goal is not None:
                fields['goal'] = {
                    'x': reached_goal[0],
                    'y': reached_goal[1],
                    'yaw': reached_goal[2],
                }
            self.publish_status(reached_state, **fields)
            self.reset_recovery()
            return

        if self.update_progress_watchdog(x, y, remaining, now):
            self.get_logger().warning(
                f'no navigation progress for {self.stuck_timeout_sec:.1f}s; '
                'starting recovery')
            self.clear_path()
            self.schedule_recovery('no_motion_progress', 0.0)
            return
        curvature = peak_path_curvature(
            self.path, self.path_index, self.curvature_lookahead_m)
        signed_curvature = signed_path_curvature(
            self.path,
            self.path_index,
            self.curvature_feedforward_lookahead_m)
        curve_speed = curvature_speed_limit(
            self.max_speed_mps,
            self.min_curve_speed_mps,
            curvature,
            self.curvature_speed_gain)
        stopping_distance = max(0.0, remaining - self.goal_tolerance_m)
        braking_speed = math.sqrt(
            2.0 * self.max_deceleration_mps2 * stopping_distance)
        target_speed = min(
            self.max_speed_mps, curve_speed, braking_speed)
        if stopping_distance > 0.025:
            target_speed = max(target_speed, self.min_speed_mps)
        final_yaw_crawl = (
            not goal_yaw_reached and
            remaining <= self.final_yaw_control_radius_m and
            path_goal_distance <= self.final_yaw_control_radius_m and
            remaining > terminal_remaining_m)
        if final_yaw_crawl:
            target_speed = max(
                target_speed,
                min(self.min_curve_speed_mps, self.max_speed_mps))
        if self.localization_status.get('state') == 'degraded':
            target_speed *= self.degraded_localization_speed_scale
        nominal_lookahead = (
            self.lookahead_min_m + self.lookahead_speed_gain * target_speed)
        lookahead = clamp(
            nominal_lookahead /
            (1.0 + self.lookahead_curvature_gain * abs(curvature)),
            self.lookahead_min_m,
            self.lookahead_max_m)
        direction = self.path_motion_direction(self.path_index)
        target_index, target_x, target_y, target_yaw = self.lookahead_target(
            x, y, lookahead, direction)
        motion_yaw = math.atan2(target_y - y, target_x - x)
        if remaining <= self.final_yaw_control_radius_m:
            target_vehicle_yaw = target_yaw
        else:
            target_vehicle_yaw = motion_yaw
            if direction < 0:
                target_vehicle_yaw = normalize_angle(motion_yaw + math.pi)
        heading_error = normalize_angle(target_vehicle_yaw - yaw)

        if self.motion_direction == 0:
            self.motion_direction = direction
            self.reset_heading_pid()

        if direction != self.motion_direction:
            commanded_speed = self.update_speed_command(0.0, now)
            steering_delta = (
                self.max_steering_rate_per_sec * self.control_period_sec)
            self.steering_command += clamp(
                -self.steering_command,
                -steering_delta,
                steering_delta)
            command = Twist()
            command.linear.x = commanded_speed
            command.angular.z = self.steering_command
            self.cmd_pub.publish(command)
            if abs(commanded_speed) <= 0.01:
                self.motion_direction = direction
                self.reset_heading_pid()
            return

        heading_abs = abs(heading_error)
        if heading_abs > self.heading_slow_angle_rad:
            target_speed *= clamp(
                self.heading_slow_angle_rad / heading_abs,
                0.25,
                1.0)

        heading_steering = direction * self.update_heading_pid(
            heading_error, now)
        cross_track_steering = self.cross_track_steering_correction(
            signed_cross_track, direction)
        curvature_feedforward = self.curvature_feedforward_gain * (
            curvature_steering_command(
                signed_curvature,
                self.wheelbase_m,
                self.max_front_wheel_angle_deg))
        raw_steering = (
            curvature_feedforward +
            heading_steering +
            cross_track_steering)
        steering = clamp(
            self.apply_directional_steering_gain(raw_steering),
            -self.max_steering_command,
            self.max_steering_command)
        commanded_speed = self.update_speed_command(
            direction * target_speed, now)
        command = Twist()
        command.linear.x = commanded_speed
        command.angular.z = steering
        self.cmd_pub.publish(command)

        if now - self.last_status_time >= self.status_period_sec:
            self.last_status_time = now
            self.get_logger().info(
                f'tracking path={self.path_index}/{len(self.path) - 1} '
                f'target={target_index} remaining={remaining:.3f}m '
                f'path_goal_distance={path_goal_distance:.3f}m '
                f'requested_goal_distance={requested_goal_distance:.3f}m '
                f'goal_yaw_error={math.degrees(goal_yaw_error):.1f}deg '
                f'cross_track={cross_track:.3f}m '
                f'signed_cross_track={signed_cross_track:.3f}m '
                f'direction={"forward" if direction > 0 else "reverse"} '
                f'heading_error={math.degrees(heading_error):.1f}deg '
                f'curvature={signed_curvature:.2f}m-1 '
                f'feedforward={curvature_feedforward:.2f} '
                f'lookahead={lookahead:.2f}m '
                f'speed={commanded_speed:.2f}/{direction * target_speed:.2f} '
                f'steering={steering:.2f} '
                f'final_yaw_crawl={final_yaw_crawl} '
                f'raw_steering={raw_steering:.2f} '
                f'heading_steering={heading_steering:.2f} '
                f'cross_track_steering={cross_track_steering:.2f}')
            self.publish_status(
                'following',
                path_index=self.path_index,
                path_size=len(self.path),
                path_mode=self.current_path_mode or None,
                approach_goal=self.current_path_approach_goal,
                relaxed_goal_yaw=self.current_path_relaxed_goal_yaw,
                remaining_m=remaining,
                goal_distance_m=requested_goal_distance,
                path_goal_distance_m=path_goal_distance,
                goal_yaw_error_deg=math.degrees(goal_yaw_error),
                requested_goal_yaw_error_deg=math.degrees(
                    requested_goal_yaw_error),
                goal_yaw_tolerance_deg=math.degrees(
                    self.goal_yaw_tolerance_rad),
                cross_track_m=cross_track,
                signed_cross_track_m=signed_cross_track,
                direction='forward' if direction > 0 else 'reverse',
                heading_error_deg=math.degrees(heading_error),
                target_speed_mps=direction * target_speed,
                speed_mps=commanded_speed,
                acceleration_mps2=self.speed_limiter.acceleration,
                steering=steering,
                final_yaw_crawl=final_yaw_crawl,
                raw_steering=raw_steering,
                left_steering_gain=self.left_steering_gain,
                right_steering_gain=self.right_steering_gain,
                heading_steering=heading_steering,
                cross_track_steering=cross_track_steering,
                curvature_feedforward=curvature_feedforward,
                signed_curvature_m_inv=signed_curvature,
                lookahead_m=lookahead,
                front_wheel_target_deg=(
                    steering * self.max_front_wheel_angle_deg),
                curvature_m_inv=curvature,
                curvature_speed_limit_mps=curve_speed,
                braking_speed_limit_mps=braking_speed,
                localization=self.localization_status,
                obstacle=self.obstacle_status,
                recovery_attempt=self.recovery_attempts,
                recovery_limit=self.max_recovery_attempts)


def main(args=None):
    rclpy.init(args=args)
    node = GoalNavigator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
