import math

from robot_lidar_bridge.board_clock import BoardClockMapper
from robot_lidar_bridge.lidar_deskew import deskew_point
from robot_lidar_bridge.tcp_laser_scan import raw_angle_to_ros_deg


def test_raw_angle_to_ros_preserves_front_and_rear():
    assert math.isclose(raw_angle_to_ros_deg(0.0), 0.0)
    assert math.isclose(raw_angle_to_ros_deg(180.0), 180.0)


def test_raw_angle_to_ros_swaps_clockwise_right_to_ros_right():
    assert math.isclose(raw_angle_to_ros_deg(90.0), 270.0)
    assert math.isclose(raw_angle_to_ros_deg(270.0), 90.0)


def test_board_clock_uses_minimum_observed_transport_delay():
    mapper = BoardClockMapper()
    first = mapper.map_ns(1_000, 11_100)
    second = mapper.map_ns(2_000, 12_050)
    assert first == 11_100
    assert second == 12_050
    third = mapper.map_ns(3_000, 13_200)
    assert third == 13_050


def test_board_clock_maps_scan_midpoint_using_scan_end_for_sync():
    mapper = BoardClockMapper()
    stamp = mapper.map_ns(
        950_000_000,
        2_010_000_000,
        sync_board_mono_ns=1_000_000_000)
    assert stamp == 1_960_000_000


def test_deskew_stationary_point_is_unchanged():
    angle, distance = deskew_point(42.0, 2.0, -0.05, 0.0, 0.0, 0.12, 0.0)
    assert math.isclose(angle, 42.0)
    assert math.isclose(distance, 2.0)


def test_deskew_forward_motion_moves_early_point_back_in_midpoint_frame():
    angle, distance = deskew_point(0.0, 2.0, -0.05, 0.2, 0.0)
    assert math.isclose(angle, 0.0)
    assert math.isclose(distance, 1.99)


def test_deskew_rotation_accounts_for_laser_offset():
    angle, distance = deskew_point(
        0.0, 2.0, 0.05, 0.0, 1.0, laser_x_m=0.12)
    assert angle > 2.0
    assert distance > 1.99
