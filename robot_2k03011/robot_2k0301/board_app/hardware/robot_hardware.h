#ifndef ROBOT_2K0301_HARDWARE_ROBOT_HARDWARE_H_
#define ROBOT_2K0301_HARDWARE_ROBOT_HARDWARE_H_

#include "control/pid.h"
#include "hardware/lidar_scanner.h"
#include "utils/config.h"

#include <utility>

namespace robot
{

class RobotHardware
{
public:
    explicit RobotHardware(const RobotConfig& config);

    bool initialize();
    void shutdown();

    void setMotorDutyPercent(double left_percent, double right_percent);
    void runEncoderTest(int seconds);
    void runEncoderRawTest(int seconds);
    void runMotorEncoderTest(int left_pwm, int right_pwm, int seconds);
    void runMotorDirectionScanTest(int pwm, int seconds_per_dir);
    void runSpeedLoopTest(double left_target, double right_target, int seconds);
    bool runStraightLidarTest(
        double left_target,
        double right_target,
        int stop_distance_mm,
        int slow_distance_mm,
        int seconds);
    bool runOdometryTcpServer(int port, bool enable_remote_drive = false);
    bool runKeyboardTeleop(double target_speed, int stop_distance_mm, int slow_distance_mm);
    bool runDistanceMove(
        bool forward,
        double distance_m,
        double target_speed,
        int stop_distance_mm,
        int slow_distance_mm,
        int timeout_seconds,
        LidarMonitorState* shared_lidar_state = nullptr);
    bool runUntilObstacle(
        bool forward,
        double max_distance_m,
        double target_speed,
        int stop_distance_mm,
        int slow_distance_mm,
        int timeout_seconds,
        LidarMonitorState* shared_lidar_state = nullptr);
    bool runAngleTurn(
        bool left,
        double angle_deg,
        double target_speed,
        int stop_distance_mm,
        int slow_distance_mm,
        int timeout_seconds,
        bool allow_initial_front_obstacle = false,
        LidarMonitorState* shared_lidar_state = nullptr);

    void motor_speed_loop(int big_state, double offset = 0.0);
    void motor_stop();

    void setSteeringServo(double angle_deg);
    void centerSteeringServo();

    void clearEncoders();
    std::pair<int, int> readAndClearEncoders();
    int read_encoder_left();
    int read_encoder_right();

    void set_motor_left_pwm(int pwm);
    void set_motor_right_pwm(int pwm);
    void set_motor_left_dir(int dir);
    void set_motor_right_dir(int dir);
    void playBeepPattern(int count, int on_ms, int off_ms);
    void beepOff();
    void calibrateImuGyroZ(bool force = false);

private:
    void stopMotors();
    void resetSpeedLoopState();
    void speedControlTick();
    double pidOutputToMotorOut(double pid_out, double speed_b, double speed_k) const;
    void resetHeadingState();
    void updateHeadingHold(double dt_seconds, double drive_direction);
    void writeSteeringServo(double angle_deg);

    RobotConfig config_;
    Pid heading_pid_;
    Pid turn_heading_pid_;
    IncrementalPid speed_pid_l_;
    IncrementalPid speed_pid_r_;
    double left_pwm_duty_max_ = 10000.0;
    double right_pwm_duty_max_ = 10000.0;
    bool car_running_ = false;
    bool manual_steering_active_ = false;

    int encoder_left_ = 0;
    int encoder_right_ = 0;
    double speed_target_l_ = 150.0;
    double speed_target_r_ = 150.0;
    double speed_real_l_ = 0.0;
    double speed_real_r_ = 0.0;
    int speed_pwm_l_ = 0;
    int speed_pwm_r_ = 0;
    double speed_dif_p_ = 0.0;
    double offset_ = 0.0;
    double speed_b_l_ = 360.0;
    double speed_b_r_ = 360.0;
    double speed_k_l_ = 40.0;
    double speed_k_r_ = 40.0;
    double speed_motor_out_l_ = 0.0;
    double speed_motor_out_r_ = 0.0;
    int speed_dir_l_ = 0;
    int speed_dir_r_ = 0;
    int speed_encoder_count_100ms_l_ = 0;
    int speed_encoder_count_100ms_r_ = 0;
    int speed_encoder_count_100ms_accum_l_ = 0;
    int speed_encoder_count_100ms_accum_r_ = 0;
    int speed_loop_100ms_sample_count_ = 0;

    bool imu_ready_ = false;
    bool imu_gyro_z_calibrated_ = false;
    double imu_gyro_z_bias_ = 0.0;
    double imu_gyro_z_raw_ = 0.0;
    double heading_yaw_error_ = 0.0;
    double heading_servo_correction_deg_ = 0.0;
    double heading_servo_angle_deg_ = 0.0;
    bool steering_servo_angle_valid_ = false;
    double steering_servo_angle_deg_ = 0.0;
};

} // namespace robot

#endif
