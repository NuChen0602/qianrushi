import math


def deskew_point(
        angle_deg, distance_m, sample_offset_sec,
        linear_velocity_mps, angular_velocity_rps,
        laser_x_m=0.0, laser_y_m=0.0):
    """Express a moving 2D lidar sample in the scan-midpoint laser frame."""
    theta = math.radians(angle_deg)
    point_x = distance_m * math.cos(theta)
    point_y = distance_m * math.sin(theta)
    yaw = angular_velocity_rps * sample_offset_sec
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)

    if abs(angular_velocity_rps) < 1.0e-9:
        base_x = linear_velocity_mps * sample_offset_sec
        base_y = 0.0
    else:
        radius = linear_velocity_mps / angular_velocity_rps
        base_x = radius * sin_yaw
        base_y = radius * (1.0 - cos_yaw)

    sensor_x = (
        base_x + cos_yaw * laser_x_m - sin_yaw * laser_y_m - laser_x_m)
    sensor_y = (
        base_y + sin_yaw * laser_x_m + cos_yaw * laser_y_m - laser_y_m)
    corrected_x = cos_yaw * point_x - sin_yaw * point_y + sensor_x
    corrected_y = sin_yaw * point_x + cos_yaw * point_y + sensor_y
    corrected_distance = math.hypot(corrected_x, corrected_y)
    corrected_angle = (
        math.degrees(math.atan2(corrected_y, corrected_x)) % 360.0)
    return corrected_angle, corrected_distance
