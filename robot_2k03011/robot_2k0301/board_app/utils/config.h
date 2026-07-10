#ifndef ROBOT_2K0301_UTILS_CONFIG_H_
#define ROBOT_2K0301_UTILS_CONFIG_H_

#include "control/pid.h"

#include <string>

namespace robot
{

struct RobotConfig
{
    int control_period_ms = 10;

    IncrementalPidConfig left_speed_loop_pid{2.0, 0.0, 0.0, 1500.0, 1500.0, 520.0, 40.0};
    IncrementalPidConfig right_speed_loop_pid{2.0, 0.0, 0.0, 1500.0, 1500.0, 520.0, 40.0};
    double speed_target_l = 150.0;
    double speed_target_r = 150.0;
    double speed_dif_p = 0.0;
    int left_speed_min_pwm = 2500;
    int right_speed_min_pwm = 2500;

    std::string left_motor_pwm = "/dev/zf_pwm_motor_1";
    std::string right_motor_pwm = "/dev/zf_pwm_motor_2";
    std::string left_motor_dir = "/dev/zf_gpio_motor_1";
    std::string right_motor_dir = "/dev/zf_gpio_motor_2";
    int left_motor_forward_dir = 0;
    int right_motor_forward_dir = 0;

    std::string left_encoder = "/dev/zf_encoder_dir_2";
    std::string right_encoder = "/dev/zf_encoder_dir_1";
    int left_encoder_sign = 1;
    int right_encoder_sign = -1;
    double left_encoder_counts_per_meter = 12812.0;
    double right_encoder_counts_per_meter = 12733.0;

    std::string steering_servo_pwm = "/dev/zf_pwm_servo_1";
    double servo_center_deg = 95.0;
    double servo_left_deg = 125.0;
    double servo_right_deg = 80.0;
    int servo_settle_ms = 300;
    double turn_inner_speed_ratio = 0.60;

    bool imu_heading_enable = true;
    double heading_KP = 2.0;
    double heading_KI = 0.0;
    double heading_KD = 0.0;
    double heading_output_limit_deg = 12.0;
    double heading_servo_dir = -1.0;
    double imu_gyro_z_scale = 0.061;
    int imu_calibration_samples = 100;
    double turn_left_angle_gain = 1.005;
    double turn_right_angle_gain = 1.005;
    double turn_heading_KP = 0.45;
    double turn_heading_KI = 0.0;
    double turn_heading_KD = 0.02;
    double turn_heading_output_limit_deg = 25.0;
    double turn_min_steering_offset_deg = 5.0;
    double turn_servo_rate_deg_per_s = 90.0;
    bool route_turn_compensation_enable = true;
    double route_left_turn_forward_compensation_m = 0.30;
    double route_right_turn_forward_compensation_m = 0.37;
    double route_min_move_distance_m = 0.03;
    bool obstacle_pause_resume_enable = true;
    int obstacle_pause_timeout_seconds = 20;
    int obstacle_resume_margin_mm = 120;
    int obstacle_stop_confirm_scans = 2;
    int obstacle_clear_confirm_scans = 5;
    int obstacle_min_pause_ms = 500;
    bool status_beep_enable = true;
    std::string beep_gpio = "/dev/zf_gpio_beep";
    int beep_active_level = 1;

    std::string lidar_serial = "/dev/ttyUSB0";
    int lidar_min_valid_mm = 0;
    double lidar_self_mask_start_deg = 120.0;
    double lidar_self_mask_end_deg = 290.0;
    int lidar_self_mask_max_mm = 350;
    int lidar_stream_port = 2368;
    int odom_stream_port = 2369;
    int remote_drive_watchdog_ms = 400;
    double remote_drive_max_speed_mps = 0.25;
    double lidar_front_center_deg = 0.0;
    double lidar_front_half_width_deg = 30.0;
    int lidar_stop_distance_mm = 500;
    int lidar_slow_distance_mm = 800;
    int lidar_turn_side_stop_distance_mm = 300;
    int lidar_turn_side_slow_distance_mm = 550;
};

class Config
{
public:
    static RobotConfig loadRobotConfig(const std::string& path);
};

} // namespace robot

#endif
