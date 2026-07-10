#include "utils/config.h"

#include "utils/logger.h"

#include <fstream>

namespace robot
{

namespace
{
std::string trim(std::string value)
{
    const auto begin = value.find_first_not_of(" \t\r\n");
    if(begin == std::string::npos)
    {
        return {};
    }
    const auto end = value.find_last_not_of(" \t\r\n");
    return value.substr(begin, end - begin + 1);
}

double toDouble(const std::string& value, double fallback)
{
    try
    {
        return std::stod(value);
    }
    catch(...)
    {
        return fallback;
    }
}

int toInt(const std::string& value, int fallback)
{
    try
    {
        return std::stoi(value);
    }
    catch(...)
    {
        return fallback;
    }
}
} // namespace

RobotConfig Config::loadRobotConfig(const std::string& path)
{
    RobotConfig config;
    std::ifstream file(path);
    if(!file.is_open())
    {
        Logger::warn("cannot open " + path + ", using default robot config");
        return config;
    }

    std::string line;
    while(std::getline(file, line))
    {
        const auto comment = line.find('#');
        if(comment != std::string::npos)
        {
            line = line.substr(0, comment);
        }

        const auto split = line.find(':');
        if(split == std::string::npos)
        {
            continue;
        }

        const auto key = trim(line.substr(0, split));
        const auto value = trim(line.substr(split + 1));
        if(key == "control_period_ms")
        {
            config.control_period_ms = toInt(value, config.control_period_ms);
        }
        else if(key == "speed_KP_l")
        {
            config.left_speed_loop_pid.kp = toDouble(value, config.left_speed_loop_pid.kp);
        }
        else if(key == "speed_KI_l")
        {
            config.left_speed_loop_pid.ki = toDouble(value, config.left_speed_loop_pid.ki);
        }
        else if(key == "speed_KD_l")
        {
            config.left_speed_loop_pid.kd = toDouble(value, config.left_speed_loop_pid.kd);
        }
        else if(key == "speed_IMAX_l")
        {
            config.left_speed_loop_pid.integral_limit = toDouble(value, config.left_speed_loop_pid.integral_limit);
        }
        else if(key == "speed_OUTMAX_l")
        {
            config.left_speed_loop_pid.output_limit = toDouble(value, config.left_speed_loop_pid.output_limit);
        }
        else if(key == "speed_target_l")
        {
            config.speed_target_l = toDouble(value, config.speed_target_l);
        }
        else if(key == "speed_b_l")
        {
            config.left_speed_loop_pid.deadzone_b = toDouble(value, config.left_speed_loop_pid.deadzone_b);
        }
        else if(key == "speed_k_l")
        {
            config.left_speed_loop_pid.scale_k = toDouble(value, config.left_speed_loop_pid.scale_k);
        }
        else if(key == "speed_KP_r")
        {
            config.right_speed_loop_pid.kp = toDouble(value, config.right_speed_loop_pid.kp);
        }
        else if(key == "speed_KI_r")
        {
            config.right_speed_loop_pid.ki = toDouble(value, config.right_speed_loop_pid.ki);
        }
        else if(key == "speed_KD_r")
        {
            config.right_speed_loop_pid.kd = toDouble(value, config.right_speed_loop_pid.kd);
        }
        else if(key == "speed_IMAX_r")
        {
            config.right_speed_loop_pid.integral_limit = toDouble(value, config.right_speed_loop_pid.integral_limit);
        }
        else if(key == "speed_OUTMAX_r")
        {
            config.right_speed_loop_pid.output_limit = toDouble(value, config.right_speed_loop_pid.output_limit);
        }
        else if(key == "speed_target_r")
        {
            config.speed_target_r = toDouble(value, config.speed_target_r);
        }
        else if(key == "speed_b_r")
        {
            config.right_speed_loop_pid.deadzone_b = toDouble(value, config.right_speed_loop_pid.deadzone_b);
        }
        else if(key == "speed_k_r")
        {
            config.right_speed_loop_pid.scale_k = toDouble(value, config.right_speed_loop_pid.scale_k);
        }
        else if(key == "speed_dif_p")
        {
            config.speed_dif_p = toDouble(value, config.speed_dif_p);
        }
        else if(key == "left_speed_min_pwm")
        {
            config.left_speed_min_pwm = toInt(value, config.left_speed_min_pwm);
        }
        else if(key == "right_speed_min_pwm")
        {
            config.right_speed_min_pwm = toInt(value, config.right_speed_min_pwm);
        }
        else if(key == "left_motor_pwm")
        {
            config.left_motor_pwm = value;
        }
        else if(key == "right_motor_pwm")
        {
            config.right_motor_pwm = value;
        }
        else if(key == "left_motor_dir")
        {
            config.left_motor_dir = value;
        }
        else if(key == "right_motor_dir")
        {
            config.right_motor_dir = value;
        }
        else if(key == "left_motor_forward_dir")
        {
            config.left_motor_forward_dir = toInt(value, config.left_motor_forward_dir) ? 1 : 0;
        }
        else if(key == "right_motor_forward_dir")
        {
            config.right_motor_forward_dir = toInt(value, config.right_motor_forward_dir) ? 1 : 0;
        }
        else if(key == "left_encoder")
        {
            config.left_encoder = value;
        }
        else if(key == "right_encoder")
        {
            config.right_encoder = value;
        }
        else if(key == "left_encoder_sign")
        {
            config.left_encoder_sign = toInt(value, config.left_encoder_sign) < 0 ? -1 : 1;
        }
        else if(key == "right_encoder_sign")
        {
            config.right_encoder_sign = toInt(value, config.right_encoder_sign) < 0 ? -1 : 1;
        }
        else if(key == "left_encoder_counts_per_meter")
        {
            config.left_encoder_counts_per_meter = toDouble(
                value, config.left_encoder_counts_per_meter);
        }
        else if(key == "right_encoder_counts_per_meter")
        {
            config.right_encoder_counts_per_meter = toDouble(
                value, config.right_encoder_counts_per_meter);
        }
        else if(key == "steering_servo_pwm")
        {
            config.steering_servo_pwm = value;
        }
        else if(key == "servo_center_deg")
        {
            config.servo_center_deg = toDouble(value, config.servo_center_deg);
        }
        else if(key == "servo_left_deg")
        {
            config.servo_left_deg = toDouble(value, config.servo_left_deg);
        }
        else if(key == "servo_right_deg")
        {
            config.servo_right_deg = toDouble(value, config.servo_right_deg);
        }
        else if(key == "servo_settle_ms")
        {
            config.servo_settle_ms = toInt(value, config.servo_settle_ms);
        }
        else if(key == "turn_inner_speed_ratio")
        {
            config.turn_inner_speed_ratio = toDouble(value, config.turn_inner_speed_ratio);
        }
        else if(key == "imu_heading_enable")
        {
            config.imu_heading_enable = toInt(value, config.imu_heading_enable ? 1 : 0) != 0;
        }
        else if(key == "heading_KP")
        {
            config.heading_KP = toDouble(value, config.heading_KP);
        }
        else if(key == "heading_KI")
        {
            config.heading_KI = toDouble(value, config.heading_KI);
        }
        else if(key == "heading_KD")
        {
            config.heading_KD = toDouble(value, config.heading_KD);
        }
        else if(key == "heading_output_limit_deg")
        {
            config.heading_output_limit_deg = toDouble(value, config.heading_output_limit_deg);
        }
        else if(key == "heading_servo_dir")
        {
            config.heading_servo_dir = toDouble(value, config.heading_servo_dir);
        }
        else if(key == "imu_gyro_z_scale")
        {
            config.imu_gyro_z_scale = toDouble(value, config.imu_gyro_z_scale);
        }
        else if(key == "imu_calibration_samples")
        {
            config.imu_calibration_samples = toInt(value, config.imu_calibration_samples);
        }
        else if(key == "turn_left_angle_gain")
        {
            config.turn_left_angle_gain = toDouble(value, config.turn_left_angle_gain);
        }
        else if(key == "turn_right_angle_gain")
        {
            config.turn_right_angle_gain = toDouble(value, config.turn_right_angle_gain);
        }
        else if(key == "turn_heading_KP")
        {
            config.turn_heading_KP = toDouble(value, config.turn_heading_KP);
        }
        else if(key == "turn_heading_KI")
        {
            config.turn_heading_KI = toDouble(value, config.turn_heading_KI);
        }
        else if(key == "turn_heading_KD")
        {
            config.turn_heading_KD = toDouble(value, config.turn_heading_KD);
        }
        else if(key == "turn_heading_output_limit_deg")
        {
            config.turn_heading_output_limit_deg = toDouble(
                value, config.turn_heading_output_limit_deg);
        }
        else if(key == "turn_min_steering_offset_deg")
        {
            config.turn_min_steering_offset_deg = toDouble(
                value, config.turn_min_steering_offset_deg);
        }
        else if(key == "turn_servo_rate_deg_per_s")
        {
            config.turn_servo_rate_deg_per_s = toDouble(
                value, config.turn_servo_rate_deg_per_s);
        }
        else if(key == "route_turn_compensation_enable")
        {
            config.route_turn_compensation_enable =
                toInt(value, config.route_turn_compensation_enable ? 1 : 0) != 0;
        }
        else if(key == "route_left_turn_forward_compensation_m")
        {
            config.route_left_turn_forward_compensation_m =
                toDouble(value, config.route_left_turn_forward_compensation_m);
        }
        else if(key == "route_right_turn_forward_compensation_m")
        {
            config.route_right_turn_forward_compensation_m =
                toDouble(value, config.route_right_turn_forward_compensation_m);
        }
        else if(key == "route_min_move_distance_m")
        {
            config.route_min_move_distance_m =
                toDouble(value, config.route_min_move_distance_m);
        }
        else if(key == "obstacle_pause_resume_enable")
        {
            config.obstacle_pause_resume_enable =
                toInt(value, config.obstacle_pause_resume_enable ? 1 : 0) != 0;
        }
        else if(key == "obstacle_pause_timeout_seconds")
        {
            config.obstacle_pause_timeout_seconds =
                toInt(value, config.obstacle_pause_timeout_seconds);
        }
        else if(key == "obstacle_resume_margin_mm")
        {
            config.obstacle_resume_margin_mm =
                toInt(value, config.obstacle_resume_margin_mm);
        }
        else if(key == "obstacle_stop_confirm_scans")
        {
            config.obstacle_stop_confirm_scans =
                toInt(value, config.obstacle_stop_confirm_scans);
        }
        else if(key == "obstacle_clear_confirm_scans")
        {
            config.obstacle_clear_confirm_scans =
                toInt(value, config.obstacle_clear_confirm_scans);
        }
        else if(key == "obstacle_min_pause_ms")
        {
            config.obstacle_min_pause_ms =
                toInt(value, config.obstacle_min_pause_ms);
        }
        else if(key == "status_beep_enable")
        {
            config.status_beep_enable =
                toInt(value, config.status_beep_enable ? 1 : 0) != 0;
        }
        else if(key == "beep_gpio")
        {
            config.beep_gpio = value;
        }
        else if(key == "beep_active_level")
        {
            config.beep_active_level = toInt(value, config.beep_active_level) ? 1 : 0;
        }
        else if(key == "lidar_serial")
        {
            config.lidar_serial = value;
        }
        else if(key == "lidar_min_valid_mm")
        {
            config.lidar_min_valid_mm = toInt(value, config.lidar_min_valid_mm);
        }
        else if(key == "lidar_self_mask_start_deg")
        {
            config.lidar_self_mask_start_deg = toDouble(value, config.lidar_self_mask_start_deg);
        }
        else if(key == "lidar_self_mask_end_deg")
        {
            config.lidar_self_mask_end_deg = toDouble(value, config.lidar_self_mask_end_deg);
        }
        else if(key == "lidar_self_mask_max_mm")
        {
            config.lidar_self_mask_max_mm = toInt(value, config.lidar_self_mask_max_mm);
        }
        else if(key == "lidar_stream_port")
        {
            config.lidar_stream_port = toInt(value, config.lidar_stream_port);
        }
        else if(key == "odom_stream_port")
        {
            config.odom_stream_port = toInt(value, config.odom_stream_port);
        }
        else if(key == "remote_drive_watchdog_ms")
        {
            config.remote_drive_watchdog_ms = toInt(value, config.remote_drive_watchdog_ms);
        }
        else if(key == "remote_drive_max_speed_mps")
        {
            config.remote_drive_max_speed_mps = toDouble(
                value, config.remote_drive_max_speed_mps);
        }
        else if(key == "lidar_front_center_deg")
        {
            config.lidar_front_center_deg = toDouble(value, config.lidar_front_center_deg);
        }
        else if(key == "lidar_front_half_width_deg")
        {
            config.lidar_front_half_width_deg = toDouble(value, config.lidar_front_half_width_deg);
        }
        else if(key == "lidar_stop_distance_mm")
        {
            config.lidar_stop_distance_mm = toInt(value, config.lidar_stop_distance_mm);
        }
        else if(key == "lidar_slow_distance_mm")
        {
            config.lidar_slow_distance_mm = toInt(value, config.lidar_slow_distance_mm);
        }
        else if(key == "lidar_turn_side_stop_distance_mm")
        {
            config.lidar_turn_side_stop_distance_mm =
                toInt(value, config.lidar_turn_side_stop_distance_mm);
        }
        else if(key == "lidar_turn_side_slow_distance_mm")
        {
            config.lidar_turn_side_slow_distance_mm =
                toInt(value, config.lidar_turn_side_slow_distance_mm);
        }
    }

    return config;
}

} // namespace robot
