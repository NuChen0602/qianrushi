#include "hardware/robot_hardware.h"

#include "hardware/lidar_scanner.h"
#include "utils/logger.h"
#include "utils/timestamp.h"

#include <algorithm>
#include <cerrno>
#include <cctype>
#include <csignal>
#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <thread>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

#if ROBOT_USE_LS2K0301_LIBRARY
#include <fcntl.h>
#include "zf_device_imu.hpp"
#include "zf_driver_encoder.hpp"
#include "zf_driver_gpio.hpp"
#include "zf_driver_pwm.hpp"
#endif

namespace robot
{

namespace
{
constexpr double kMaxBringupMotorPercent = 30.0;

double clampMotorPercent(double value)
{
    return std::clamp(value, -kMaxBringupMotorPercent, kMaxBringupMotorPercent);
}

int clampPwmDuty(int value, double duty_max)
{
    return static_cast<int>(std::clamp(static_cast<double>(value), 0.0, duty_max));
}

int inverted(int value)
{
    return value ? 0 : 1;
}

bool sendAll(int socket_fd, const std::string& data)
{
    std::size_t sent = 0;
    while(sent < data.size())
    {
        const auto result = send(
            socket_fd,
            data.data() + sent,
            data.size() - sent,
            MSG_NOSIGNAL);
        if(result <= 0)
        {
            return false;
        }
        sent += static_cast<std::size_t>(result);
    }
    return true;
}

class RawTerminal
{
public:
    RawTerminal()
    {
        if(tcgetattr(STDIN_FILENO, &original_) != 0)
        {
            return;
        }
        termios raw = original_;
        cfmakeraw(&raw);
        raw.c_oflag |= OPOST;
        active_ = tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0;
    }

    ~RawTerminal()
    {
        if(active_)
        {
            tcsetattr(STDIN_FILENO, TCSANOW, &original_);
        }
    }

    bool active() const
    {
        return active_;
    }

private:
    termios original_ {};
    bool active_ = false;
};

volatile std::sig_atomic_t g_action_stop_requested = 0;

void actionStopSignalHandler(int)
{
    g_action_stop_requested = 1;
}

class ActionSignalGuard
{
public:
    ActionSignalGuard()
    {
        g_action_stop_requested = 0;
        previous_int_ = std::signal(SIGINT, actionStopSignalHandler);
        previous_term_ = std::signal(SIGTERM, actionStopSignalHandler);
        previous_hup_ = std::signal(SIGHUP, actionStopSignalHandler);
    }

    ~ActionSignalGuard()
    {
        std::signal(SIGINT, previous_int_);
        std::signal(SIGTERM, previous_term_);
        std::signal(SIGHUP, previous_hup_);
    }

    bool stopRequested() const
    {
        return g_action_stop_requested != 0;
    }

private:
    using SignalHandler = void (*)(int);
    SignalHandler previous_int_ = SIG_DFL;
    SignalHandler previous_term_ = SIG_DFL;
    SignalHandler previous_hup_ = SIG_DFL;
};

#if ROBOT_USE_LS2K0301_LIBRARY
class DirectPwmMotorDriver
{
public:
    explicit DirectPwmMotorDriver(const RobotConfig& config)
        : left_dir_(config.left_motor_dir.c_str(), O_RDWR),
          right_dir_(config.right_motor_dir.c_str(), O_RDWR),
          left_pwm_(config.left_motor_pwm.c_str()),
          right_pwm_(config.right_motor_pwm.c_str())
    {
        left_pwm_.get_dev_info(&left_info_);
        right_pwm_.get_dev_info(&right_info_);
        if(left_info_.duty_max == 0)
        {
            left_info_.duty_max = 10000;
        }
        if(right_info_.duty_max == 0)
        {
            right_info_.duty_max = 10000;
        }
    }

    void stop()
    {
        left_pwm_.set_duty(0);
        right_pwm_.set_duty(0);
    }

    void setLeftDirection(int dir)
    {
        left_dir_.set_level(dir ? 1 : 0);
    }

    void setRightDirection(int dir)
    {
        right_dir_.set_level(dir ? 1 : 0);
    }

    void setLeftDuty(int duty)
    {
        left_pwm_.set_duty(static_cast<uint16>(clampPwmDuty(duty, leftDutyMax())));
    }

    void setRightDuty(int duty)
    {
        right_pwm_.set_duty(static_cast<uint16>(clampPwmDuty(duty, rightDutyMax())));
    }

    double leftDutyMax() const
    {
        return static_cast<double>(left_info_.duty_max);
    }

    double rightDutyMax() const
    {
        return static_cast<double>(right_info_.duty_max);
    }

private:
    zf_driver_gpio left_dir_;
    zf_driver_gpio right_dir_;
    zf_driver_pwm left_pwm_;
    zf_driver_pwm right_pwm_;
    pwm_info left_info_ {};
    pwm_info right_info_ {};
};

class EncoderDriver
{
public:
    explicit EncoderDriver(const RobotConfig& config)
        : left_(config.left_encoder.c_str()),
          right_(config.right_encoder.c_str()),
          left_sign_(config.left_encoder_sign),
          right_sign_(config.right_encoder_sign)
    {
    }

    void clear()
    {
        left_.clear_count();
        right_.clear_count();
    }

    std::pair<int, int> readRaw()
    {
        return {static_cast<int>(left_.get_count()), static_cast<int>(right_.get_count())};
    }

    std::pair<int, int> readSigned()
    {
        const auto [left, right] = readRaw();
        return {left_sign_ * left, right_sign_ * right};
    }

    std::pair<int, int> readAndClear()
    {
        const auto counts = readSigned();
        clear();
        return counts;
    }

    std::pair<int, int> readRawAndClear()
    {
        const auto counts = readRaw();
        clear();
        return counts;
    }

private:
    zf_driver_encoder left_;
    zf_driver_encoder right_;
    int left_sign_ = 1;
    int right_sign_ = 1;
};

class SteeringServoDriver
{
public:
    explicit SteeringServoDriver(const RobotConfig& config)
        : servo_(config.steering_servo_pwm.c_str())
    {
        servo_.get_dev_info(&info_);
    }

    void setAngle(double angle_deg)
    {
        servo_.set_duty(dutyFromAngle(angle_deg));
    }

    void release()
    {
        servo_.set_duty(0);
    }

private:
    uint16 dutyFromAngle(double angle_deg) const
    {
        angle_deg = std::clamp(angle_deg, 45.0, 135.0);
        if(info_.freq == 0 || info_.duty_max == 0)
        {
            return 0;
        }

        const double pulse_ms = 0.5 + angle_deg / 90.0;
        const double period_ms = 1000.0 / static_cast<double>(info_.freq);
        const double duty = static_cast<double>(info_.duty_max) * pulse_ms / period_ms;
        return static_cast<uint16>(std::clamp(duty, 0.0, static_cast<double>(info_.duty_max)));
    }

    zf_driver_pwm servo_;
    pwm_info info_ {};
};

class BeepDriver
{
public:
    explicit BeepDriver(const RobotConfig& config)
        : beep_(config.beep_gpio.c_str(), O_RDWR),
          active_level_(config.beep_active_level ? 1 : 0)
    {
    }

    void on()
    {
        beep_.set_level(static_cast<uint8>(active_level_));
    }

    void off()
    {
        beep_.set_level(static_cast<uint8>(active_level_ ? 0 : 1));
    }

private:
    zf_driver_gpio beep_;
    int active_level_ = 1;
};

class ImuDriver
{
public:
    bool init()
    {
        const auto type = imu_.init();
        ready_ = type == DEV_IMU660RA || type == DEV_IMU660RB || type == DEV_IMU963RA;
        return ready_;
    }

    int gyroZ()
    {
        return ready_ ? static_cast<int>(imu_.get_gyro_z()) : 0;
    }

    int type() const
    {
        return static_cast<int>(imu_.imu_type);
    }

private:
    zf_device_imu imu_;
    bool ready_ = false;
};

std::unique_ptr<DirectPwmMotorDriver> g_motor;
std::unique_ptr<EncoderDriver> g_encoders;
std::unique_ptr<SteeringServoDriver> g_servo;
std::unique_ptr<BeepDriver> g_beep;
std::unique_ptr<ImuDriver> g_imu;
#endif
} // namespace

RobotHardware::RobotHardware(const RobotConfig& config)
    : config_(config),
      heading_pid_({config.heading_KP, config.heading_KI, config.heading_KD, config.heading_output_limit_deg, config.heading_output_limit_deg}),
      turn_heading_pid_({config.turn_heading_KP, config.turn_heading_KI, config.turn_heading_KD, config.turn_heading_output_limit_deg, config.turn_heading_output_limit_deg}),
      speed_pid_l_(config.left_speed_loop_pid),
      speed_pid_r_(config.right_speed_loop_pid),
      speed_target_l_(config.speed_target_l),
      speed_target_r_(config.speed_target_r),
      speed_dif_p_(config.speed_dif_p),
      speed_b_l_(config.left_speed_loop_pid.deadzone_b),
      speed_b_r_(config.right_speed_loop_pid.deadzone_b),
      speed_k_l_(config.left_speed_loop_pid.scale_k),
      speed_k_r_(config.right_speed_loop_pid.scale_k)
{
}

bool RobotHardware::initialize()
{
#if ROBOT_USE_LS2K0301_LIBRARY
    Logger::info("initializing LS2K0301 motor/encoder/imu/servo hardware");

    g_motor = std::make_unique<DirectPwmMotorDriver>(config_);
    g_encoders = std::make_unique<EncoderDriver>(config_);
    g_servo = std::make_unique<SteeringServoDriver>(config_);
    g_beep = std::make_unique<BeepDriver>(config_);
    g_beep->off();

    left_pwm_duty_max_ = g_motor->leftDutyMax();
    right_pwm_duty_max_ = g_motor->rightDutyMax();

    std::ostringstream motor_log;
    motor_log << "direct pwm motor mode"
              << " left_pwm=" << config_.left_motor_pwm
              << " right_pwm=" << config_.right_motor_pwm
              << " left_dir=" << config_.left_motor_dir
              << " right_dir=" << config_.right_motor_dir
              << " left_duty_max=" << left_pwm_duty_max_
              << " right_duty_max=" << right_pwm_duty_max_;
    Logger::info(motor_log.str());

    if(config_.imu_heading_enable)
    {
        g_imu = std::make_unique<ImuDriver>();
        imu_ready_ = g_imu->init();
        std::ostringstream imu_log;
        imu_log << "imu heading hold " << (imu_ready_ ? "enabled" : "unavailable")
                << " imu_type=" << (g_imu ? g_imu->type() : 0);
        Logger::info(imu_log.str());
    }

    clearEncoders();
    stopMotors();
    centerSteeringServo();
    return true;
#else
    Logger::warn("running without LS2K0301 library; hardware output is disabled");
    return true;
#endif
}

void RobotHardware::shutdown()
{
    motor_stop();
    centerSteeringServo();
    beepOff();
}

void RobotHardware::setMotorDutyPercent(double left_percent, double right_percent)
{
    const double left = clampMotorPercent(left_percent);
    const double right = clampMotorPercent(right_percent);
    if(left != left_percent || right != right_percent)
    {
        Logger::warn("motor duty is limited to +/-30% during bringup");
    }

    const int left_dir = left >= 0.0 ? config_.left_motor_forward_dir : inverted(config_.left_motor_forward_dir);
    const int right_dir = right >= 0.0 ? config_.right_motor_forward_dir : inverted(config_.right_motor_forward_dir);
    const int left_duty = clampPwmDuty(static_cast<int>(std::abs(left) * left_pwm_duty_max_ / 100.0), left_pwm_duty_max_);
    const int right_duty = clampPwmDuty(static_cast<int>(std::abs(right) * right_pwm_duty_max_ / 100.0), right_pwm_duty_max_);

    set_motor_left_dir(left_dir);
    set_motor_right_dir(right_dir);
    set_motor_left_pwm(left_duty);
    set_motor_right_pwm(right_duty);
}

void RobotHardware::runEncoderTest(int seconds)
{
    Logger::info("encoder test starting");
    clearEncoders();

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(std::max(1, seconds));
    int sample = 0;
    int left_total = 0;
    int right_total = 0;
    while(std::chrono::steady_clock::now() < deadline)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        const auto [left, right] = readAndClearEncoders();
        left_total += left;
        right_total += right;

        std::ostringstream oss;
        oss << "encoder sample=" << sample++
            << " left_delta=" << left
            << " right_delta=" << right
            << " left_total=" << left_total
            << " right_total=" << right_total;
        Logger::info(oss.str());
    }
    Logger::info("encoder test stopped");
}

void RobotHardware::runEncoderRawTest(int seconds)
{
    Logger::info("encoder raw test starting");
    clearEncoders();

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(std::max(1, seconds));
    int sample = 0;
    int raw_left_total = 0;
    int raw_right_total = 0;
    int signed_left_total = 0;
    int signed_right_total = 0;
    while(std::chrono::steady_clock::now() < deadline)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(std::max(1, config_.control_period_ms)));

        int raw_left = 0;
        int raw_right = 0;
#if ROBOT_USE_LS2K0301_LIBRARY
        if(g_encoders)
        {
            const auto [left, right] = g_encoders->readRawAndClear();
            raw_left = left;
            raw_right = right;
        }
#endif
        const int signed_left = config_.left_encoder_sign * raw_left;
        const int signed_right = config_.right_encoder_sign * raw_right;
        raw_left_total += raw_left;
        raw_right_total += raw_right;
        signed_left_total += signed_left;
        signed_right_total += signed_right;

        std::ostringstream oss;
        oss << "encoder_raw sample=" << sample++
            << " raw_left=" << raw_left
            << " raw_right=" << raw_right
            << " signed_left=" << signed_left
            << " signed_right=" << signed_right
            << " raw_left_total=" << raw_left_total
            << " raw_right_total=" << raw_right_total
            << " signed_left_total=" << signed_left_total
            << " signed_right_total=" << signed_right_total
            << " left_node=" << config_.left_encoder
            << " right_node=" << config_.right_encoder
            << " left_sign=" << config_.left_encoder_sign
            << " right_sign=" << config_.right_encoder_sign;
        Logger::info(oss.str());
    }

    Logger::info("encoder raw test stopped");
}

void RobotHardware::runMotorEncoderTest(int left_pwm, int right_pwm, int seconds)
{
    Logger::info("motor encoder fixed-pwm test starting");
    centerSteeringServo();

    const int left_dir = left_pwm >= 0 ? config_.left_motor_forward_dir : inverted(config_.left_motor_forward_dir);
    const int right_dir = right_pwm >= 0 ? config_.right_motor_forward_dir : inverted(config_.right_motor_forward_dir);
    const int left_duty = clampPwmDuty(std::abs(left_pwm), left_pwm_duty_max_);
    const int right_duty = clampPwmDuty(std::abs(right_pwm), right_pwm_duty_max_);

    clearEncoders();
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(std::max(1, seconds));
    int sample = 0;
    int left_total = 0;
    int right_total = 0;
    int left_100ms = 0;
    int right_100ms = 0;
    int left_100ms_accum = 0;
    int right_100ms_accum = 0;
    int tick_count = 0;

    while(std::chrono::steady_clock::now() < deadline)
    {
        const auto cycle_start = std::chrono::steady_clock::now();

        set_motor_left_dir(left_dir);
        set_motor_right_dir(right_dir);
        set_motor_left_pwm(left_duty);
        set_motor_right_pwm(right_duty);

        const auto [left_delta, right_delta] = readAndClearEncoders();
        left_total += left_delta;
        right_total += right_delta;
        left_100ms_accum += left_delta;
        right_100ms_accum += right_delta;
        tick_count++;
        if(tick_count >= 10)
        {
            left_100ms = left_100ms_accum;
            right_100ms = right_100ms_accum;
            left_100ms_accum = 0;
            right_100ms_accum = 0;
            tick_count = 0;
        }

        if(sample++ % 10 == 0)
        {
            std::ostringstream oss;
            oss << "motor_encoder sample=" << sample
                << " left_cmd_pwm=" << left_pwm
                << " right_cmd_pwm=" << right_pwm
                << " left_duty=" << left_duty
                << " right_duty=" << right_duty
                << " left_dir=" << left_dir
                << " right_dir=" << right_dir
                << " left_delta_10ms=" << left_delta
                << " right_delta_10ms=" << right_delta
                << " left_count_100ms=" << left_100ms
                << " right_count_100ms=" << right_100ms
                << " left_total=" << left_total
                << " right_total=" << right_total;
            Logger::info(oss.str());
        }

        std::this_thread::sleep_until(cycle_start + std::chrono::milliseconds(10));
    }

    motor_stop();
    Logger::info("motor encoder fixed-pwm test stopped");
}

void RobotHardware::runMotorDirectionScanTest(int pwm, int seconds_per_dir)
{
    Logger::info("motor direction scan test starting");
    centerSteeringServo();

    const int duty = clampPwmDuty(std::abs(pwm), std::min(left_pwm_duty_max_, right_pwm_duty_max_));
    const int run_seconds = std::max(1, seconds_per_dir);

    for(int dir = 0; dir <= 1; ++dir)
    {
        clearEncoders();
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(run_seconds);
        int sample = 0;
        int left_total = 0;
        int right_total = 0;

        std::ostringstream start_log;
        start_log << "motor direction scan phase dir=" << dir
                  << " duty=" << duty
                  << " seconds=" << run_seconds;
        Logger::info(start_log.str());

        while(std::chrono::steady_clock::now() < deadline)
        {
            const auto cycle_start = std::chrono::steady_clock::now();
            set_motor_left_dir(dir);
            set_motor_right_dir(dir);
            set_motor_left_pwm(duty);
            set_motor_right_pwm(duty);

            const auto [left_delta, right_delta] = readAndClearEncoders();
            left_total += left_delta;
            right_total += right_delta;

            if(sample++ % 10 == 0)
            {
                std::ostringstream oss;
                oss << "motor_dir_scan dir=" << dir
                    << " duty=" << duty
                    << " left_delta_10ms=" << left_delta
                    << " right_delta_10ms=" << right_delta
                    << " left_total=" << left_total
                    << " right_total=" << right_total;
                Logger::info(oss.str());
            }

            std::this_thread::sleep_until(cycle_start + std::chrono::milliseconds(10));
        }

        motor_stop();
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }

    Logger::info("motor direction scan test stopped");
}

void RobotHardware::runSpeedLoopTest(double left_target, double right_target, int seconds)
{
    Logger::info("speed closed-loop test starting");
    centerSteeringServo();

    speed_target_l_ = left_target;
    speed_target_r_ = right_target;
    offset_ = 0.0;
    car_running_ = true;
    resetSpeedLoopState();
    resetHeadingState();
    calibrateImuGyroZ();
    clearEncoders();

    const auto period = std::chrono::milliseconds(10);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(std::max(1, seconds));
    int sample = 0;

    while(std::chrono::steady_clock::now() < deadline)
    {
        const auto cycle_start = std::chrono::steady_clock::now();
        motor_speed_loop(1, offset_);

        if(sample++ % 10 == 0)
        {
            std::ostringstream oss;
            oss << "speed_loop sample=" << sample
                << " left_target=" << speed_target_l_
                << " right_target=" << speed_target_r_
                << " left_speed=" << speed_real_l_
                << " right_speed=" << speed_real_r_
                << " left_pwm=" << speed_pwm_l_
                << " right_pwm=" << speed_pwm_r_
                << " left_dir=" << speed_dir_l_
                << " right_dir=" << speed_dir_r_
                << " left_motor_out=" << speed_motor_out_l_
                << " right_motor_out=" << speed_motor_out_r_
                << " left_encoder_count_10ms=" << encoder_left_
                << " right_encoder_count_10ms=" << encoder_right_
                << " left_encoder_count_100ms=" << speed_encoder_count_100ms_l_
                << " right_encoder_count_100ms=" << speed_encoder_count_100ms_r_
                << " left_pid=" << speed_pid_l_.output()
                << " right_pid=" << speed_pid_r_.output()
                << " imu_gyro_z=" << imu_gyro_z_raw_
                << " imu_gyro_z_bias=" << imu_gyro_z_bias_
                << " heading_yaw_error=" << heading_yaw_error_
                << " servo_correction_deg=" << heading_servo_correction_deg_
                << " servo_angle_deg=" << heading_servo_angle_deg_;
            Logger::info(oss.str());
        }

        std::this_thread::sleep_until(cycle_start + period);
    }

    motor_speed_loop(0, 0.0);
    Logger::info("speed closed-loop test stopped");
}

bool RobotHardware::runStraightLidarTest(
    double left_target,
    double right_target,
    int stop_distance_mm,
    int slow_distance_mm,
    int seconds)
{
    const int stop_mm = std::max(100, stop_distance_mm);
    const int slow_mm = std::max(stop_mm + 100, slow_distance_mm);
    const double base_left_target = left_target;
    const double base_right_target = right_target;

    LidarScanner lidar(
        config_.lidar_serial,
        config_.lidar_min_valid_mm,
        config_.lidar_self_mask_start_deg,
        config_.lidar_self_mask_end_deg,
        config_.lidar_self_mask_max_mm);
    LidarMonitorState lidar_state;
    std::thread lidar_thread([&]() {
        lidar.monitorFrontSector(
            lidar_state,
            config_.lidar_front_center_deg,
            config_.lidar_front_half_width_deg);
    });

    const auto ready_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while(!lidar_state.ready && std::chrono::steady_clock::now() < ready_deadline)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    const auto first_scan_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while(lidar_state.ready && !lidar_state.failed && lidar_state.scan_count == 0 &&
          std::chrono::steady_clock::now() < first_scan_deadline)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    if(!lidar_state.ready || lidar_state.failed || lidar_state.scan_count == 0)
    {
        Logger::error("straight lidar test aborted: lidar is not ready");
        motor_stop();
        lidar_state.running = false;
        if(lidar_thread.joinable())
        {
            lidar_thread.join();
        }
        return false;
    }

    std::ostringstream start_log;
    start_log << "straight lidar test starting"
              << " front_center_deg=" << config_.lidar_front_center_deg
              << " front_half_width_deg=" << config_.lidar_front_half_width_deg
              << " stop_mm=" << stop_mm
              << " slow_mm=" << slow_mm;
    Logger::info(start_log.str());

    centerSteeringServo();
    speed_target_l_ = base_left_target;
    speed_target_r_ = base_right_target;
    offset_ = 0.0;
    car_running_ = true;
    resetSpeedLoopState();
    resetHeadingState();
    calibrateImuGyroZ();
    clearEncoders();

    const auto period = std::chrono::milliseconds(10);
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::seconds(std::max(1, seconds));
    auto last_scan_time = std::chrono::steady_clock::now();
    uint64_t last_scan_count = lidar_state.scan_count.load();
    int front_distance_mm = lidar_state.front_distance_mm.load();
    bool lidar_ok = true;
    int sample = 0;

    while(std::chrono::steady_clock::now() < deadline)
    {
        const auto cycle_start = std::chrono::steady_clock::now();
        const uint64_t scan_count = lidar_state.scan_count.load();
        if(scan_count != last_scan_count)
        {
            const int new_front_distance_mm = lidar_state.front_distance_mm.load();
            if(front_distance_mm > 0 && front_distance_mm < slow_mm &&
               new_front_distance_mm > front_distance_mm + 300)
            {
                std::ostringstream stop_log;
                stop_log << "straight lidar emergency stop: nearby obstacle track lost"
                         << " previous_mm=" << front_distance_mm
                         << " current_mm=" << new_front_distance_mm;
                Logger::warn(stop_log.str());
                break;
            }
            front_distance_mm = new_front_distance_mm;
            last_scan_count = scan_count;
            last_scan_time = cycle_start;
        }

        if(lidar_state.failed || cycle_start - last_scan_time > std::chrono::milliseconds(600))
        {
            Logger::error("straight lidar emergency stop: lidar data timeout");
            lidar_ok = false;
            break;
        }

        if(front_distance_mm > 0 && front_distance_mm <= stop_mm)
        {
            std::ostringstream stop_log;
            stop_log << "straight lidar obstacle stop front_distance_mm=" << front_distance_mm
                     << " stop_mm=" << stop_mm;
            Logger::warn(stop_log.str());
            break;
        }

        double speed_scale = 1.0;
        if(front_distance_mm > stop_mm && front_distance_mm < slow_mm)
        {
            const double range_scale = static_cast<double>(front_distance_mm - stop_mm) /
                                       static_cast<double>(slow_mm - stop_mm);
            speed_scale = std::clamp(0.35 + 0.65 * range_scale, 0.35, 1.0);
        }

        speed_target_l_ = base_left_target * speed_scale;
        speed_target_r_ = base_right_target * speed_scale;
        motor_speed_loop(1, offset_);

        if(sample++ % 10 == 0)
        {
            std::ostringstream oss;
            oss << "straight_lidar sample=" << sample
                << " front_distance_mm=" << front_distance_mm
                << " speed_scale=" << speed_scale
                << " left_target=" << speed_target_l_
                << " right_target=" << speed_target_r_
                << " left_speed=" << speed_real_l_
                << " right_speed=" << speed_real_r_
                << " left_pwm=" << speed_pwm_l_
                << " right_pwm=" << speed_pwm_r_
                << " heading_yaw_error=" << heading_yaw_error_
                << " servo_angle_deg=" << heading_servo_angle_deg_;
            Logger::info(oss.str());
        }

        std::this_thread::sleep_until(cycle_start + period);
    }

    motor_speed_loop(0, 0.0);
    speed_target_l_ = base_left_target;
    speed_target_r_ = base_right_target;
    lidar_state.running = false;
    if(lidar_thread.joinable())
    {
        lidar_thread.join();
    }
    Logger::info("straight lidar test stopped");
    return lidar_ok;
}

bool RobotHardware::runKeyboardTeleop(
    double target_speed,
    int stop_distance_mm,
    int slow_distance_mm)
{
    const int stop_mm = std::max(100, stop_distance_mm);
    const int slow_mm = std::max(stop_mm + 100, slow_distance_mm);
    double selected_speed = std::clamp(std::abs(target_speed), 5.0, 100.0);

    LidarScanner lidar(
        config_.lidar_serial,
        config_.lidar_min_valid_mm,
        config_.lidar_self_mask_start_deg,
        config_.lidar_self_mask_end_deg,
        config_.lidar_self_mask_max_mm);
    LidarMonitorState lidar_state;
    std::thread lidar_thread([&]() {
        lidar.monitorFrontSector(
            lidar_state,
            config_.lidar_front_center_deg,
            config_.lidar_front_half_width_deg);
    });

    const auto ready_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while(!lidar_state.ready && std::chrono::steady_clock::now() < ready_deadline)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    const auto first_scan_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while(lidar_state.ready && !lidar_state.failed && lidar_state.scan_count == 0 &&
          std::chrono::steady_clock::now() < first_scan_deadline)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if(!lidar_state.ready || lidar_state.failed || lidar_state.scan_count == 0)
    {
        Logger::error("keyboard teleop aborted: lidar is not ready");
        motor_stop();
        lidar_state.running = false;
        if(lidar_thread.joinable())
        {
            lidar_thread.join();
        }
        return false;
    }

    RawTerminal terminal;
    if(!terminal.active())
    {
        Logger::warn("stdin is not a TTY; run keyboard teleop through ssh -t");
    }

    Logger::info("keyboard teleop ready: hold W/S, A/D steer, C center, +/- speed, SPACE stop, Q quit");
    manual_steering_active_ = true;
    centerSteeringServo();
    double steering_angle = config_.servo_center_deg;
    double drive_command = 0.0;
    bool obstacle_latched = false;
    bool success = true;
    bool running = true;
    resetSpeedLoopState();
    clearEncoders();

    const auto period = std::chrono::milliseconds(10);
    auto last_drive_key = std::chrono::steady_clock::now();
    auto last_scan_time = last_drive_key;
    uint64_t last_scan_count = lidar_state.scan_count.load();
    int status_tick = 0;

    while(running)
    {
        const auto cycle_start = std::chrono::steady_clock::now();
        fd_set read_set;
        FD_ZERO(&read_set);
        FD_SET(STDIN_FILENO, &read_set);
        timeval timeout {};
        const int ready = select(STDIN_FILENO + 1, &read_set, nullptr, nullptr, &timeout);
        if(ready > 0 && FD_ISSET(STDIN_FILENO, &read_set))
        {
            char keys[32];
            const ssize_t count = read(STDIN_FILENO, keys, sizeof(keys));
            if(count <= 0)
            {
                Logger::warn("keyboard teleop connection closed");
                break;
            }
            for(ssize_t i = 0; i < count; ++i)
            {
                const unsigned char raw_key = static_cast<unsigned char>(keys[i]);
                const char key = static_cast<char>(std::tolower(raw_key));
                if(key == 'w')
                {
                    drive_command = selected_speed;
                    last_drive_key = cycle_start;
                    obstacle_latched = false;
                }
                else if(key == 's')
                {
                    drive_command = -selected_speed;
                    last_drive_key = cycle_start;
                    obstacle_latched = false;
                }
                else if(key == 'a')
                {
                    steering_angle = std::min(
                        steering_angle + 2.0,
                        std::max(config_.servo_left_deg, config_.servo_right_deg));
                }
                else if(key == 'd')
                {
                    steering_angle = std::max(
                        steering_angle - 2.0,
                        std::min(config_.servo_left_deg, config_.servo_right_deg));
                }
                else if(key == 'c')
                {
                    steering_angle = config_.servo_center_deg;
                }
                else if(key == '+' || key == '=')
                {
                    selected_speed = std::min(selected_speed + 5.0, 100.0);
                }
                else if(key == '-' || key == '_')
                {
                    selected_speed = std::max(selected_speed - 5.0, 5.0);
                }
                else if(key == ' ' || key == 'x')
                {
                    drive_command = 0.0;
                }
                else if(key == 'q' || raw_key == 3)
                {
                    drive_command = 0.0;
                    running = false;
                }
            }
#if ROBOT_USE_LS2K0301_LIBRARY
            if(g_servo)
            {
                g_servo->setAngle(steering_angle);
            }
#endif
        }

        if(drive_command != 0.0 &&
           cycle_start - last_drive_key > std::chrono::milliseconds(650))
        {
            drive_command = 0.0;
            Logger::warn("keyboard watchdog stopped motors; keep W or S pressed to move");
        }

        const uint64_t scan_count = lidar_state.scan_count.load();
        if(scan_count != last_scan_count)
        {
            last_scan_count = scan_count;
            last_scan_time = cycle_start;
        }
        if(lidar_state.failed || cycle_start - last_scan_time > std::chrono::milliseconds(600))
        {
            Logger::error("keyboard teleop emergency stop: lidar data timeout");
            success = false;
            break;
        }

        const int front_mm = lidar_state.front_distance_mm.load();
        const int rear_mm = lidar_state.rear_distance_mm.load();
        const int motion_distance_mm = drive_command >= 0.0 ? front_mm : rear_mm;
        double applied_speed = drive_command;
        if(drive_command != 0.0 && motion_distance_mm <= stop_mm)
        {
            applied_speed = 0.0;
            if(!obstacle_latched)
            {
                std::ostringstream oss;
                oss << "keyboard teleop obstacle stop direction="
                    << (drive_command > 0.0 ? "front" : "rear")
                    << " distance_mm=" << motion_distance_mm;
                Logger::warn(oss.str());
                obstacle_latched = true;
            }
        }
        else if(drive_command != 0.0 && motion_distance_mm < slow_mm)
        {
            const double scale = std::clamp(
                static_cast<double>(motion_distance_mm - stop_mm) /
                    static_cast<double>(slow_mm - stop_mm),
                0.25,
                1.0);
            applied_speed *= scale;
        }

        speed_target_l_ = applied_speed;
        speed_target_r_ = applied_speed;
        motor_speed_loop(applied_speed == 0.0 ? 0 : 1, 0.0);

        if(status_tick++ % 20 == 0)
        {
            std::ostringstream oss;
            oss << "teleop speed=" << applied_speed
                << " selected_speed=" << selected_speed
                << " steering_deg=" << steering_angle
                << " front_mm=" << front_mm
                << " rear_mm=" << rear_mm
                << " left_real=" << speed_real_l_
                << " right_real=" << speed_real_r_;
            Logger::info(oss.str());
        }
        std::this_thread::sleep_until(cycle_start + period);
    }

    motor_stop();
    manual_steering_active_ = false;
    lidar_state.running = false;
    if(lidar_thread.joinable())
    {
        lidar_thread.join();
    }
    centerSteeringServo();
    Logger::info("keyboard teleop stopped");
    return success;
}

bool RobotHardware::runDistanceMove(
    bool forward,
    double distance_m,
    double target_speed,
    int stop_distance_mm,
    int slow_distance_mm,
    int timeout_seconds,
    LidarMonitorState* shared_lidar_state)
{
    ActionSignalGuard signal_guard;
    if(distance_m <= 0.0)
    {
        Logger::error("distance move requires a positive distance");
        return false;
    }

    const int stop_mm = std::max(100, stop_distance_mm);
    const int slow_mm = std::max(stop_mm + 100, slow_distance_mm);
    const double direction = forward ? 1.0 : -1.0;
    const double base_speed = std::clamp(std::abs(target_speed), 5.0, 100.0);
    const double left_counts_per_meter = std::max(
        1.0, config_.left_encoder_counts_per_meter);
    const double right_counts_per_meter = std::max(
        1.0, config_.right_encoder_counts_per_meter);

    std::unique_ptr<LidarScanner> owned_lidar;
    LidarMonitorState owned_lidar_state;
    LidarMonitorState& lidar_state = shared_lidar_state
        ? *shared_lidar_state
        : owned_lidar_state;
    std::thread lidar_thread;
    if(!shared_lidar_state)
    {
        owned_lidar = std::make_unique<LidarScanner>(
            config_.lidar_serial,
            config_.lidar_min_valid_mm,
            config_.lidar_self_mask_start_deg,
            config_.lidar_self_mask_end_deg,
            config_.lidar_self_mask_max_mm);
        lidar_thread = std::thread([&]() {
            owned_lidar->monitorFrontSector(
                lidar_state,
                config_.lidar_front_center_deg,
                config_.lidar_front_half_width_deg);
        });
    }

    const auto ready_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while(!lidar_state.ready && std::chrono::steady_clock::now() < ready_deadline)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    const auto first_scan_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while(lidar_state.ready && !lidar_state.failed && lidar_state.scan_count == 0 &&
          std::chrono::steady_clock::now() < first_scan_deadline)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if(!lidar_state.ready || lidar_state.failed || lidar_state.scan_count == 0)
    {
        Logger::error("distance move aborted: lidar is not ready");
        if(!shared_lidar_state)
        {
            lidar_state.running = false;
            if(lidar_thread.joinable())
            {
                lidar_thread.join();
            }
        }
        return false;
    }

    centerSteeringServo();
    manual_steering_active_ = false;
    resetSpeedLoopState();
    resetHeadingState();
    calibrateImuGyroZ();
    clearEncoders();

    std::ostringstream start_log;
    start_log << "distance move starting direction=" << (forward ? "forward" : "backward")
              << " target_m=" << distance_m
              << " speed=" << base_speed;
    Logger::info(start_log.str());

    long long left_total = 0;
    long long right_total = 0;
    double traveled_m = 0.0;
    bool completed = false;
    bool sensor_ok = true;
    const auto period = std::chrono::milliseconds(10);
    auto deadline = std::chrono::steady_clock::now() +
                    std::chrono::seconds(std::max(1, timeout_seconds));
    auto last_scan_time = std::chrono::steady_clock::now();
    uint64_t last_scan_count = lidar_state.scan_count.load();
    int log_tick = 0;
    int no_motion_ticks = 0;
    int obstacle_pause_count = 0;
    int obstacle_hit_scans = 0;
    const int obstacle_stop_confirm_scans =
        std::max(1, config_.obstacle_stop_confirm_scans);
    const int obstacle_clear_confirm_scans =
        std::max(1, config_.obstacle_clear_confirm_scans);
    const auto obstacle_min_pause =
        std::chrono::milliseconds(std::max(0, config_.obstacle_min_pause_ms));

    while(std::chrono::steady_clock::now() < deadline && !signal_guard.stopRequested())
    {
        const auto cycle_start = std::chrono::steady_clock::now();
        const uint64_t scan_count = lidar_state.scan_count.load();
        bool new_scan = false;
        if(scan_count != last_scan_count)
        {
            last_scan_count = scan_count;
            last_scan_time = cycle_start;
            new_scan = true;
        }
        if(lidar_state.failed || cycle_start - last_scan_time > std::chrono::milliseconds(600))
        {
            Logger::error("distance move emergency stop: lidar data timeout");
            sensor_ok = false;
            break;
        }

        const int obstacle_mm = forward
            ? lidar_state.front_distance_mm.load()
            : lidar_state.rear_distance_mm.load();
        if(new_scan)
        {
            if(obstacle_mm <= stop_mm)
            {
                ++obstacle_hit_scans;
            }
            else
            {
                obstacle_hit_scans = 0;
            }
        }
        if(obstacle_hit_scans >= obstacle_stop_confirm_scans)
        {
            if(!config_.obstacle_pause_resume_enable)
            {
                std::ostringstream oss;
                oss << "distance move obstacle stop distance_mm=" << obstacle_mm;
                Logger::warn(oss.str());
                break;
            }

            motor_stop();
            resetSpeedLoopState();
            ++obstacle_pause_count;
            const auto pause_start = cycle_start;
            const auto pause_deadline = pause_start +
                std::chrono::seconds(std::max(1, config_.obstacle_pause_timeout_seconds));
            const int resume_mm = stop_mm + std::max(20, config_.obstacle_resume_margin_mm);
            int current_obstacle_mm = obstacle_mm;
            bool obstacle_cleared = false;
            int clear_scans = 0;

            std::ostringstream pause_log;
            pause_log << "distance move obstacle pause count=" << obstacle_pause_count
                      << " distance_mm=" << obstacle_mm
                      << " resume_mm=" << resume_mm
                      << " stop_confirm_scans=" << obstacle_stop_confirm_scans
                      << " clear_confirm_scans=" << obstacle_clear_confirm_scans
                      << " min_pause_ms=" << config_.obstacle_min_pause_ms
                      << " timeout_s=" << config_.obstacle_pause_timeout_seconds;
            Logger::warn(pause_log.str());
            playBeepPattern(2, 120, 80);

            while(std::chrono::steady_clock::now() < pause_deadline &&
                  !signal_guard.stopRequested())
            {
                const auto wait_now = std::chrono::steady_clock::now();
                const uint64_t wait_scan_count = lidar_state.scan_count.load();
                if(wait_scan_count != last_scan_count)
                {
                    last_scan_count = wait_scan_count;
                    last_scan_time = wait_now;
                    current_obstacle_mm = forward
                        ? lidar_state.front_distance_mm.load()
                        : lidar_state.rear_distance_mm.load();
                    if(current_obstacle_mm > resume_mm)
                    {
                        ++clear_scans;
                    }
                    else
                    {
                        clear_scans = 0;
                    }
                }
                if(lidar_state.failed ||
                   wait_now - last_scan_time > std::chrono::milliseconds(600))
                {
                    Logger::error("distance move emergency stop during obstacle pause: lidar data timeout");
                    sensor_ok = false;
                    break;
                }
                if(clear_scans >= obstacle_clear_confirm_scans &&
                   wait_now - pause_start >= obstacle_min_pause)
                {
                    obstacle_cleared = true;
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }

            const auto pause_end = std::chrono::steady_clock::now();
            deadline += pause_end - pause_start;
            if(!sensor_ok || signal_guard.stopRequested())
            {
                break;
            }
            if(!obstacle_cleared)
            {
                std::ostringstream timeout_log;
                timeout_log << "distance move obstacle pause timeout distance_mm="
                            << current_obstacle_mm;
                Logger::warn(timeout_log.str());
                sensor_ok = false;
                break;
            }

            std::ostringstream resume_log;
            resume_log << "distance move obstacle cleared distance_mm="
                       << current_obstacle_mm
                       << ", resuming";
            Logger::info(resume_log.str());
            playBeepPattern(1, 120, 0);
            resetSpeedLoopState();
            no_motion_ticks = 0;
            obstacle_hit_scans = 0;
            continue;
        }

        const double remaining_m = std::max(0.0, distance_m - traveled_m);
        double endpoint_scale = std::clamp(remaining_m / 0.20, 0.25, 1.0);
        double obstacle_scale = 1.0;
        if(obstacle_mm < slow_mm)
        {
            obstacle_scale = std::clamp(
                static_cast<double>(obstacle_mm - stop_mm) /
                    static_cast<double>(slow_mm - stop_mm),
                0.25,
                1.0);
        }
        const double command_speed = direction * base_speed *
                                     std::min(endpoint_scale, obstacle_scale);
        speed_target_l_ = command_speed;
        speed_target_r_ = command_speed;
        motor_speed_loop(1, 0.0);

        if(std::abs(encoder_left_) + std::abs(encoder_right_) == 0)
        {
            ++no_motion_ticks;
        }
        else
        {
            no_motion_ticks = 0;
        }
        if(no_motion_ticks >= 200)
        {
            std::ostringstream oss;
            oss << "distance move no-motion stop speed=" << command_speed
                << " left_pwm=" << speed_pwm_l_
                << " right_pwm=" << speed_pwm_r_
                << " left_motor_out=" << speed_motor_out_l_
                << " right_motor_out=" << speed_motor_out_r_;
            Logger::error(oss.str());
            sensor_ok = false;
            break;
        }

        left_total += encoder_left_;
        right_total += encoder_right_;
        const double left_m = direction * static_cast<double>(left_total) /
                              left_counts_per_meter;
        const double right_m = direction * static_cast<double>(right_total) /
                               right_counts_per_meter;
        traveled_m = std::max(0.0, 0.5 * (left_m + right_m));
        if(traveled_m >= distance_m)
        {
            completed = true;
            break;
        }

        if(log_tick++ % 20 == 0)
        {
            std::ostringstream oss;
            oss << "distance_move traveled_m=" << traveled_m
                << " target_m=" << distance_m
                << " remaining_m=" << std::max(0.0, distance_m - traveled_m)
                << " obstacle_mm=" << obstacle_mm
                << " speed=" << command_speed
                << " left_pwm=" << speed_pwm_l_
                << " right_pwm=" << speed_pwm_r_
                << " left_dir=" << speed_dir_l_
                << " right_dir=" << speed_dir_r_
                << " left_motor_out=" << speed_motor_out_l_
                << " right_motor_out=" << speed_motor_out_r_
                << " left_delta=" << encoder_left_
                << " right_delta=" << encoder_right_
                << " left_count=" << left_total
                << " right_count=" << right_total;
            Logger::info(oss.str());
        }
        std::this_thread::sleep_until(cycle_start + period);
    }

    motor_stop();
    if(!shared_lidar_state)
    {
        lidar_state.running = false;
        if(lidar_thread.joinable())
        {
            lidar_thread.join();
        }
    }
    centerSteeringServo();

    std::ostringstream result_log;
    result_log << "distance move stopped completed=" << completed
               << " traveled_m=" << traveled_m
               << " obstacle_pauses=" << obstacle_pause_count;
    Logger::info(result_log.str());
    if(signal_guard.stopRequested())
    {
        Logger::warn("distance move interrupted by signal");
    }
    return completed && sensor_ok;
}

bool RobotHardware::runUntilObstacle(
    bool forward,
    double max_distance_m,
    double target_speed,
    int stop_distance_mm,
    int slow_distance_mm,
    int timeout_seconds,
    LidarMonitorState* shared_lidar_state)
{
    ActionSignalGuard signal_guard;
    if(max_distance_m <= 0.0)
    {
        Logger::error("move until obstacle requires a positive max distance");
        return false;
    }

    const int stop_mm = std::max(100, stop_distance_mm);
    const int slow_mm = std::max(stop_mm + 100, slow_distance_mm);
    const double direction = forward ? 1.0 : -1.0;
    const double base_speed = std::clamp(std::abs(target_speed), 5.0, 100.0);
    const double left_counts_per_meter = std::max(
        1.0, config_.left_encoder_counts_per_meter);
    const double right_counts_per_meter = std::max(
        1.0, config_.right_encoder_counts_per_meter);

    std::unique_ptr<LidarScanner> owned_lidar;
    LidarMonitorState owned_lidar_state;
    LidarMonitorState& lidar_state = shared_lidar_state
        ? *shared_lidar_state
        : owned_lidar_state;
    std::thread lidar_thread;
    if(!shared_lidar_state)
    {
        owned_lidar = std::make_unique<LidarScanner>(
            config_.lidar_serial,
            config_.lidar_min_valid_mm,
            config_.lidar_self_mask_start_deg,
            config_.lidar_self_mask_end_deg,
            config_.lidar_self_mask_max_mm);
        lidar_thread = std::thread([&]() {
            owned_lidar->monitorFrontSector(
                lidar_state,
                config_.lidar_front_center_deg,
                config_.lidar_front_half_width_deg);
        });
    }

    const auto ready_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while(!lidar_state.ready && std::chrono::steady_clock::now() < ready_deadline)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    const auto first_scan_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while(lidar_state.ready && !lidar_state.failed && lidar_state.scan_count == 0 &&
          std::chrono::steady_clock::now() < first_scan_deadline)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if(!lidar_state.ready || lidar_state.failed || lidar_state.scan_count == 0)
    {
        Logger::error("move until obstacle aborted: lidar is not ready");
        if(!shared_lidar_state)
        {
            lidar_state.running = false;
            if(lidar_thread.joinable())
            {
                lidar_thread.join();
            }
        }
        return false;
    }

    centerSteeringServo();
    manual_steering_active_ = false;
    resetSpeedLoopState();
    resetHeadingState();
    calibrateImuGyroZ();
    clearEncoders();

    std::ostringstream start_log;
    start_log << "move until obstacle starting direction="
              << (forward ? "forward" : "backward")
              << " max_distance_m=" << max_distance_m
              << " speed=" << base_speed
              << " stop_mm=" << stop_mm
              << " slow_mm=" << slow_mm;
    Logger::info(start_log.str());

    long long left_total = 0;
    long long right_total = 0;
    double traveled_m = 0.0;
    bool obstacle_detected = false;
    bool sensor_ok = true;
    bool limit_reached = false;
    const auto period = std::chrono::milliseconds(10);
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::seconds(std::max(1, timeout_seconds));
    auto last_scan_time = std::chrono::steady_clock::now();
    uint64_t last_scan_count = lidar_state.scan_count.load();
    int obstacle_hit_scans = 0;
    int no_motion_ticks = 0;
    int log_tick = 0;
    const int obstacle_stop_confirm_scans =
        std::max(1, config_.obstacle_stop_confirm_scans);

    while(std::chrono::steady_clock::now() < deadline && !signal_guard.stopRequested())
    {
        const auto cycle_start = std::chrono::steady_clock::now();
        const uint64_t scan_count = lidar_state.scan_count.load();
        bool new_scan = false;
        if(scan_count != last_scan_count)
        {
            last_scan_count = scan_count;
            last_scan_time = cycle_start;
            new_scan = true;
        }
        if(lidar_state.failed || cycle_start - last_scan_time > std::chrono::milliseconds(600))
        {
            Logger::error("move until obstacle emergency stop: lidar data timeout");
            sensor_ok = false;
            break;
        }

        const int obstacle_mm = forward
            ? lidar_state.front_distance_mm.load()
            : lidar_state.rear_distance_mm.load();
        if(new_scan)
        {
            if(obstacle_mm <= stop_mm)
            {
                ++obstacle_hit_scans;
            }
            else
            {
                obstacle_hit_scans = 0;
            }
        }
        if(obstacle_hit_scans >= obstacle_stop_confirm_scans)
        {
            obstacle_detected = true;
            std::ostringstream obstacle_log;
            obstacle_log << "move until obstacle detected distance_mm=" << obstacle_mm
                         << " traveled_m=" << traveled_m
                         << " confirm_scans=" << obstacle_hit_scans;
            Logger::warn(obstacle_log.str());
            break;
        }

        const double remaining_limit_m = std::max(0.0, max_distance_m - traveled_m);
        if(remaining_limit_m <= 0.0)
        {
            limit_reached = true;
            break;
        }

        const double endpoint_scale = std::clamp(remaining_limit_m / 0.20, 0.25, 1.0);
        double obstacle_scale = 1.0;
        if(obstacle_mm <= stop_mm)
        {
            obstacle_scale = 0.20;
        }
        else if(obstacle_mm < slow_mm)
        {
            obstacle_scale = std::clamp(
                static_cast<double>(obstacle_mm - stop_mm) /
                    static_cast<double>(slow_mm - stop_mm),
                0.25,
                1.0);
        }
        const double command_speed = direction * base_speed *
                                     std::min(endpoint_scale, obstacle_scale);
        speed_target_l_ = command_speed;
        speed_target_r_ = command_speed;
        motor_speed_loop(1, 0.0);

        if(std::abs(encoder_left_) + std::abs(encoder_right_) == 0)
        {
            ++no_motion_ticks;
        }
        else
        {
            no_motion_ticks = 0;
        }
        if(no_motion_ticks >= 200)
        {
            Logger::error("move until obstacle no-motion stop");
            sensor_ok = false;
            break;
        }

        left_total += encoder_left_;
        right_total += encoder_right_;
        const double left_m = direction * static_cast<double>(left_total) /
                              left_counts_per_meter;
        const double right_m = direction * static_cast<double>(right_total) /
                               right_counts_per_meter;
        traveled_m = std::max(0.0, 0.5 * (left_m + right_m));
        if(traveled_m >= max_distance_m)
        {
            limit_reached = true;
            break;
        }

        if(log_tick++ % 20 == 0)
        {
            std::ostringstream status_log;
            status_log << "move_until_obstacle traveled_m=" << traveled_m
                       << " max_distance_m=" << max_distance_m
                       << " obstacle_mm=" << obstacle_mm
                       << " obstacle_hits=" << obstacle_hit_scans
                       << " speed=" << command_speed
                       << " left_pwm=" << speed_pwm_l_
                       << " right_pwm=" << speed_pwm_r_
                       << " left_count=" << left_total
                       << " right_count=" << right_total;
            Logger::info(status_log.str());
        }
        std::this_thread::sleep_until(cycle_start + period);
    }

    motor_stop();
    if(!shared_lidar_state)
    {
        lidar_state.running = false;
        if(lidar_thread.joinable())
        {
            lidar_thread.join();
        }
    }
    centerSteeringServo();

    if(obstacle_detected)
    {
        playBeepPattern(1, 60, 0);
    }
    else if(limit_reached)
    {
        Logger::error("move until obstacle safety distance reached before detecting obstacle");
    }
    else if(sensor_ok && !signal_guard.stopRequested())
    {
        Logger::error("move until obstacle timed out before detecting obstacle");
    }

    std::ostringstream result_log;
    result_log << "move until obstacle stopped detected=" << obstacle_detected
               << " traveled_m=" << traveled_m
               << " limit_reached=" << limit_reached;
    Logger::info(result_log.str());
    if(signal_guard.stopRequested())
    {
        Logger::warn("move until obstacle interrupted by signal");
    }
    return obstacle_detected && sensor_ok && !signal_guard.stopRequested();
}

bool RobotHardware::runAngleTurn(
    bool left,
    double angle_deg,
    double target_speed,
    int stop_distance_mm,
    int slow_distance_mm,
    int timeout_seconds,
    bool allow_initial_front_obstacle,
    LidarMonitorState* shared_lidar_state)
{
    ActionSignalGuard signal_guard;
    if(angle_deg <= 0.0)
    {
        Logger::error("angle turn requires a positive angle");
        return false;
    }
#if ROBOT_USE_LS2K0301_LIBRARY
    if(!imu_ready_ || !g_imu)
    {
        Logger::error("angle turn aborted: IMU is unavailable");
        return false;
    }
#endif

    const int stop_mm = std::max(100, stop_distance_mm);
    const int slow_mm = std::max(stop_mm + 100, slow_distance_mm);
    const int side_stop_mm = std::clamp(
        config_.lidar_turn_side_stop_distance_mm,
        100,
        stop_mm);
    const int side_slow_mm = std::max(
        side_stop_mm + 100,
        config_.lidar_turn_side_slow_distance_mm);
    const double base_speed = std::clamp(std::abs(target_speed), 5.0, 60.0);
    const double requested_angle = std::clamp(std::abs(angle_deg), 1.0, 360.0);
    const double angle_gain = std::clamp(
        left ? config_.turn_left_angle_gain : config_.turn_right_angle_gain,
        0.50,
        2.00);
    const double target_angle = std::clamp(requested_angle * angle_gain, 1.0, 360.0);
    const double inner_speed_ratio = std::clamp(config_.turn_inner_speed_ratio, 0.20, 0.95);

    std::unique_ptr<LidarScanner> owned_lidar;
    LidarMonitorState owned_lidar_state;
    LidarMonitorState& lidar_state = shared_lidar_state
        ? *shared_lidar_state
        : owned_lidar_state;
    std::thread lidar_thread;
    if(!shared_lidar_state)
    {
        owned_lidar = std::make_unique<LidarScanner>(
            config_.lidar_serial,
            config_.lidar_min_valid_mm,
            config_.lidar_self_mask_start_deg,
            config_.lidar_self_mask_end_deg,
            config_.lidar_self_mask_max_mm);
        lidar_thread = std::thread([&]() {
            owned_lidar->monitorFrontSector(
                lidar_state,
                config_.lidar_front_center_deg,
                config_.lidar_front_half_width_deg);
        });
    }

    const auto ready_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while(!lidar_state.ready && std::chrono::steady_clock::now() < ready_deadline)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    const auto first_scan_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while(lidar_state.ready && !lidar_state.failed && lidar_state.scan_count == 0 &&
          std::chrono::steady_clock::now() < first_scan_deadline)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if(!lidar_state.ready || lidar_state.failed || lidar_state.scan_count == 0)
    {
        Logger::error("angle turn aborted: lidar is not ready");
        if(!shared_lidar_state)
        {
            lidar_state.running = false;
            if(lidar_thread.joinable())
            {
                lidar_thread.join();
            }
        }
        return false;
    }

    motor_stop();
    resetSpeedLoopState();
    resetHeadingState();
    calibrateImuGyroZ();
    clearEncoders();
    manual_steering_active_ = true;
    turn_heading_pid_.reset();
    double steering_angle = steering_servo_angle_valid_
        ? steering_servo_angle_deg_
        : config_.servo_center_deg;
    steering_angle = std::clamp(
        steering_angle,
        std::min(config_.servo_left_deg, config_.servo_right_deg),
        std::max(config_.servo_left_deg, config_.servo_right_deg));
    const double turn_direction = left ? -1.0 : 1.0;
    const double signed_target_angle = turn_direction * target_angle;

    std::ostringstream start_log;
    start_log << "angle turn starting direction=" << (left ? "left" : "right")
              << " requested_deg=" << requested_angle
              << " control_target_deg=" << target_angle
              << " angle_gain=" << angle_gain
              << " steering_start_deg=" << steering_angle
              << " turn_pid=(" << config_.turn_heading_KP
              << ',' << config_.turn_heading_KI
              << ',' << config_.turn_heading_KD << ')'
              << " turn_pid_limit_deg=" << config_.turn_heading_output_limit_deg
              << " min_steering_offset_deg=" << config_.turn_min_steering_offset_deg
              << " servo_rate_deg_per_s=" << config_.turn_servo_rate_deg_per_s
              << " inner_speed_ratio=" << inner_speed_ratio
              << " allow_initial_front_obstacle=" << allow_initial_front_obstacle
              << " speed=" << base_speed;
    Logger::info(start_log.str());

    double yaw_total_deg = 0.0;
    double turned_deg = 0.0;
    bool completed = false;
    bool sensor_ok = true;
    const auto period = std::chrono::milliseconds(10);
    auto deadline = std::chrono::steady_clock::now() +
                    std::chrono::seconds(std::max(1, timeout_seconds));
    auto previous_time = std::chrono::steady_clock::now();
    auto last_scan_time = previous_time;
    uint64_t last_scan_count = lidar_state.scan_count.load();
    int log_tick = 0;
    long long left_turn_count = 0;
    long long right_turn_count = 0;
    int obstacle_pause_count = 0;
    int obstacle_hit_scans = 0;
    bool front_guard_enabled = !allow_initial_front_obstacle;
    const int obstacle_stop_confirm_scans =
        std::max(1, config_.obstacle_stop_confirm_scans);
    const int obstacle_clear_confirm_scans =
        std::max(1, config_.obstacle_clear_confirm_scans);
    const auto obstacle_min_pause =
        std::chrono::milliseconds(std::max(0, config_.obstacle_min_pause_ms));

    while(std::chrono::steady_clock::now() < deadline && !signal_guard.stopRequested())
    {
        const auto cycle_start = std::chrono::steady_clock::now();
        const double dt_seconds = std::clamp(
            std::chrono::duration<double>(cycle_start - previous_time).count(),
            0.001,
            0.05);
        previous_time = cycle_start;

        const uint64_t scan_count = lidar_state.scan_count.load();
        bool new_scan = false;
        if(scan_count != last_scan_count)
        {
            last_scan_count = scan_count;
            last_scan_time = cycle_start;
            new_scan = true;
        }
        if(lidar_state.failed || cycle_start - last_scan_time > std::chrono::milliseconds(600))
        {
            Logger::error("angle turn emergency stop: lidar data timeout");
            sensor_ok = false;
            break;
        }

        const int front_mm = lidar_state.front_distance_mm.load();
        const int side_mm = left
            ? lidar_state.left_distance_mm.load()
            : lidar_state.right_distance_mm.load();
        if(!front_guard_enabled &&
           (front_mm > stop_mm + std::max(20, config_.obstacle_resume_margin_mm) ||
            turned_deg >= 45.0))
        {
            front_guard_enabled = true;
            std::ostringstream guard_log;
            guard_log << "angle turn front obstacle guard restored"
                      << " turned_deg=" << turned_deg
                      << " front_mm=" << front_mm;
            Logger::info(guard_log.str());
        }
        const bool front_stop = front_guard_enabled && front_mm <= stop_mm;
        const bool side_stop = side_mm <= side_stop_mm;
        if(new_scan)
        {
            if(front_stop || side_stop)
            {
                ++obstacle_hit_scans;
            }
            else
            {
                obstacle_hit_scans = 0;
            }
        }
        if(obstacle_hit_scans >= obstacle_stop_confirm_scans)
        {
            if(!config_.obstacle_pause_resume_enable)
            {
                std::ostringstream oss;
                oss << "angle turn obstacle stop source="
                    << (front_stop ? "front" : "side")
                    << " front_mm=" << front_mm
                    << " side_mm=" << side_mm
                    << " front_stop_mm=" << stop_mm
                    << " side_stop_mm=" << side_stop_mm;
                Logger::warn(oss.str());
                break;
            }

            motor_stop();
            resetSpeedLoopState();
            ++obstacle_pause_count;
            const auto pause_start = cycle_start;
            const auto pause_deadline = pause_start +
                std::chrono::seconds(std::max(1, config_.obstacle_pause_timeout_seconds));
            const int front_resume_mm = stop_mm +
                std::max(20, config_.obstacle_resume_margin_mm);
            const int side_resume_mm = side_stop_mm +
                std::max(20, config_.obstacle_resume_margin_mm);
            int current_front_mm = front_mm;
            int current_side_mm = side_mm;
            bool obstacle_cleared = false;
            int clear_scans = 0;

            std::ostringstream pause_log;
            pause_log << "angle turn obstacle pause count=" << obstacle_pause_count
                      << " source=" << (front_stop ? "front" : "side")
                      << " front_mm=" << front_mm
                      << " side_mm=" << side_mm
                      << " front_resume_mm=" << front_resume_mm
                      << " side_resume_mm=" << side_resume_mm
                      << " stop_confirm_scans=" << obstacle_stop_confirm_scans
                      << " clear_confirm_scans=" << obstacle_clear_confirm_scans
                      << " min_pause_ms=" << config_.obstacle_min_pause_ms
                      << " timeout_s=" << config_.obstacle_pause_timeout_seconds;
            Logger::warn(pause_log.str());
            playBeepPattern(2, 120, 80);

            while(std::chrono::steady_clock::now() < pause_deadline &&
                  !signal_guard.stopRequested())
            {
                const auto wait_now = std::chrono::steady_clock::now();
                const uint64_t wait_scan_count = lidar_state.scan_count.load();
                if(wait_scan_count != last_scan_count)
                {
                    last_scan_count = wait_scan_count;
                    last_scan_time = wait_now;
                    current_front_mm = lidar_state.front_distance_mm.load();
                    current_side_mm = left
                        ? lidar_state.left_distance_mm.load()
                        : lidar_state.right_distance_mm.load();
                    if(current_front_mm > front_resume_mm && current_side_mm > side_resume_mm)
                    {
                        ++clear_scans;
                    }
                    else
                    {
                        clear_scans = 0;
                    }
                }
                if(lidar_state.failed ||
                   wait_now - last_scan_time > std::chrono::milliseconds(600))
                {
                    Logger::error("angle turn emergency stop during obstacle pause: lidar data timeout");
                    sensor_ok = false;
                    break;
                }
                if(clear_scans >= obstacle_clear_confirm_scans &&
                   wait_now - pause_start >= obstacle_min_pause)
                {
                    obstacle_cleared = true;
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }

            const auto pause_end = std::chrono::steady_clock::now();
            deadline += pause_end - pause_start;
            previous_time = pause_end;
            if(!sensor_ok || signal_guard.stopRequested())
            {
                break;
            }
            if(!obstacle_cleared)
            {
                std::ostringstream timeout_log;
                timeout_log << "angle turn obstacle pause timeout"
                            << " front_mm=" << current_front_mm
                            << " side_mm=" << current_side_mm;
                Logger::warn(timeout_log.str());
                sensor_ok = false;
                break;
            }

            std::ostringstream resume_log;
            resume_log << "angle turn obstacle cleared front_mm="
                       << current_front_mm
                       << " side_mm=" << current_side_mm
                       << ", resuming";
            Logger::info(resume_log.str());
            playBeepPattern(1, 120, 0);
            resetSpeedLoopState();
            obstacle_hit_scans = 0;
            continue;
        }

        double gyro_z_raw = 0.0;
        double gyro_z_dps = 0.0;
#if ROBOT_USE_LS2K0301_LIBRARY
        gyro_z_raw = static_cast<double>(g_imu->gyroZ());
        gyro_z_dps = (gyro_z_raw - imu_gyro_z_bias_) * config_.imu_gyro_z_scale;
#endif
        if(std::abs(gyro_z_dps) < 0.1)
        {
            gyro_z_dps = 0.0;
        }
        yaw_total_deg += gyro_z_dps * dt_seconds;
        turned_deg = std::abs(yaw_total_deg);
        if(turned_deg >= target_angle)
        {
            completed = true;
            break;
        }

        const double remaining_deg = std::max(0.0, target_angle - turned_deg);
        const double turn_pid_output = turn_heading_pid_.update(
            signed_target_angle,
            yaw_total_deg,
            dt_seconds);
        double desired_steering_offset = turn_pid_output * config_.heading_servo_dir;
        const double min_steering_offset = std::max(
            0.0,
            std::abs(config_.turn_min_steering_offset_deg));
        if(remaining_deg > 0.5)
        {
            if(left)
            {
                desired_steering_offset = std::max(
                    desired_steering_offset,
                    min_steering_offset);
            }
            else
            {
                desired_steering_offset = std::min(
                    desired_steering_offset,
                    -min_steering_offset);
            }
        }
        const double desired_steering_angle = std::clamp(
            config_.servo_center_deg + desired_steering_offset,
            std::min(config_.servo_left_deg, config_.servo_right_deg),
            std::max(config_.servo_left_deg, config_.servo_right_deg));
        const double max_servo_step = std::max(
            0.1,
            std::abs(config_.turn_servo_rate_deg_per_s) * dt_seconds);
        steering_angle += std::clamp(
            desired_steering_angle - steering_angle,
            -max_servo_step,
            max_servo_step);
        writeSteeringServo(steering_angle);

        const double endpoint_scale = std::clamp(remaining_deg / 20.0, 0.30, 1.0);
        double front_scale = 1.0;
        if(front_guard_enabled && front_mm < slow_mm)
        {
            front_scale = std::clamp(
                static_cast<double>(front_mm - stop_mm) /
                    static_cast<double>(slow_mm - stop_mm),
                0.25,
                1.0);
        }
        double side_scale = 1.0;
        if(side_mm < side_slow_mm)
        {
            side_scale = std::clamp(
                static_cast<double>(side_mm - side_stop_mm) /
                    static_cast<double>(side_slow_mm - side_stop_mm),
                0.25,
                1.0);
        }
        const double obstacle_scale = std::min(front_scale, side_scale);
        const double outer_speed = base_speed * std::min(endpoint_scale, obstacle_scale);
        const double inner_speed = outer_speed * inner_speed_ratio;
        speed_target_l_ = left ? inner_speed : outer_speed;
        speed_target_r_ = left ? outer_speed : inner_speed;
        motor_speed_loop(1, 0.0);
        left_turn_count += encoder_left_;
        right_turn_count += encoder_right_;

        if(log_tick++ % 20 == 0)
        {
            const double left_distance_m =
                std::abs(static_cast<double>(left_turn_count)) /
                std::max(1.0, config_.left_encoder_counts_per_meter);
            const double right_distance_m =
                std::abs(static_cast<double>(right_turn_count)) /
                std::max(1.0, config_.right_encoder_counts_per_meter);
            std::ostringstream oss;
            oss << "angle_turn turned_deg=" << turned_deg
                << " target_deg=" << target_angle
                << " yaw_total_deg=" << yaw_total_deg
                << " gyro_z_raw=" << gyro_z_raw
                << " gyro_z_dps=" << gyro_z_dps
                << " gyro_z_scale=" << config_.imu_gyro_z_scale
                << " heading_error_deg=" << (signed_target_angle - yaw_total_deg)
                << " turn_pid_out=" << turn_pid_output
                << " steering_target_deg=" << desired_steering_angle
                << " steering_deg=" << steering_angle
                << " front_mm=" << front_mm
                << " side_mm=" << side_mm
                << " front_guard=" << front_guard_enabled
                << " left_target=" << speed_target_l_
                << " right_target=" << speed_target_r_
                << " path_m=" << (left_distance_m + right_distance_m) * 0.5
                << " left_count=" << left_turn_count
                << " right_count=" << right_turn_count;
            Logger::info(oss.str());
        }
        std::this_thread::sleep_until(cycle_start + period);
    }

    motor_stop();
    manual_steering_active_ = false;
    turn_heading_pid_.reset();
    if(!shared_lidar_state)
    {
        lidar_state.running = false;
        if(lidar_thread.joinable())
        {
            lidar_thread.join();
        }
    }
    writeSteeringServo(config_.servo_center_deg);

    const double left_distance_m =
        std::abs(static_cast<double>(left_turn_count)) /
        std::max(1.0, config_.left_encoder_counts_per_meter);
    const double right_distance_m =
        std::abs(static_cast<double>(right_turn_count)) /
        std::max(1.0, config_.right_encoder_counts_per_meter);

    std::ostringstream result_log;
    result_log << "angle turn stopped completed=" << completed
               << " requested_deg=" << requested_angle
               << " control_target_deg=" << target_angle
               << " turned_deg=" << turned_deg
               << " path_m=" << (left_distance_m + right_distance_m) * 0.5
               << " left_distance_m=" << left_distance_m
               << " right_distance_m=" << right_distance_m
               << " obstacle_pauses=" << obstacle_pause_count;
    Logger::info(result_log.str());
    if(signal_guard.stopRequested())
    {
        Logger::warn("angle turn interrupted by signal");
    }
    return completed && sensor_ok;
}

bool RobotHardware::runOdometryTcpServer(int port, bool enable_remote_drive)
{
    ActionSignalGuard signal_guard;
    motor_stop();
    calibrateImuGyroZ();

    const int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if(server_fd < 0)
    {
        Logger::error("cannot create odometry TCP socket");
        return false;
    }

    const int reuse = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in address {};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = htons(static_cast<uint16_t>(std::clamp(port, 1, 65535)));
    if(bind(server_fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0 ||
       listen(server_fd, 1) < 0)
    {
        Logger::error("cannot bind odometry TCP port " + std::to_string(port));
        close(server_fd);
        return false;
    }

    std::ostringstream listen_log;
    listen_log << "odometry TCP stream listening on port " << port
               << " remote_drive=" << enable_remote_drive;
    Logger::info(listen_log.str());

    const auto apply_servo_without_delay = [&](double angle_deg) {
        const double clamped_angle = std::clamp(
            angle_deg,
            std::min(config_.servo_left_deg, config_.servo_right_deg),
            std::max(config_.servo_left_deg, config_.servo_right_deg));
#if ROBOT_USE_LS2K0301_LIBRARY
        if(g_servo)
        {
            g_servo->setAngle(clamped_angle);
        }
#endif
        steering_servo_angle_deg_ = clamped_angle;
        steering_servo_angle_valid_ = true;
    };

    const auto watchdog_timeout = std::chrono::milliseconds(
        std::clamp(config_.remote_drive_watchdog_ms, 100, 2000));
    const double max_drive_speed_mps = std::clamp(
        std::abs(config_.remote_drive_max_speed_mps), 0.05, 1.0);
    const double control_period_seconds = 0.01;
    uint64_t sequence = 0;
    while(!signal_guard.stopRequested())
    {
        fd_set accept_set;
        FD_ZERO(&accept_set);
        FD_SET(server_fd, &accept_set);
        timeval accept_timeout {};
        accept_timeout.tv_usec = 200000;
        const int accept_ready = select(
            server_fd + 1, &accept_set, nullptr, nullptr, &accept_timeout);
        if(accept_ready <= 0 || !FD_ISSET(server_fd, &accept_set))
        {
            continue;
        }
        const int client_fd = accept(server_fd, nullptr, nullptr);
        if(client_fd < 0)
        {
            continue;
        }

        Logger::info(enable_remote_drive
            ? "odometry TCP client connected; remote drive armed with zero command"
            : "odometry TCP client connected");
        motor_stop();
        resetSpeedLoopState();
        manual_steering_active_ = enable_remote_drive;
        clearEncoders();
        apply_servo_without_delay(config_.servo_center_deg);

        std::string command_buffer;
        double command_speed_mps = 0.0;
        double command_steering_deg = config_.servo_center_deg;
        bool command_received = false;
        bool watchdog_reported = false;
        bool client_connected = true;
        auto last_command_time = std::chrono::steady_clock::now();
        auto previous_odom_time = last_command_time;
        auto next_odom_time = last_command_time;
        int left_odom_accum = 0;
        int right_odom_accum = 0;
        int status_tick = 0;
        bool heading_hold_active = false;
        double heading_hold_direction = 0.0;

        while(client_connected && !signal_guard.stopRequested())
        {
            const auto cycle_start = std::chrono::steady_clock::now();

            if(enable_remote_drive)
            {
                char receive_buffer[256];
                while(true)
                {
                    const ssize_t received = recv(
                        client_fd,
                        receive_buffer,
                        sizeof(receive_buffer),
                        MSG_DONTWAIT);
                    if(received > 0)
                    {
                        command_buffer.append(
                            receive_buffer, static_cast<std::size_t>(received));
                        continue;
                    }
                    if(received == 0)
                    {
                        client_connected = false;
                    }
                    else if(errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
                    {
                        client_connected = false;
                    }
                    break;
                }

                if(command_buffer.size() > 4096)
                {
                    Logger::warn("remote drive command buffer overflow; stopping motors");
                    command_buffer.clear();
                    command_speed_mps = 0.0;
                    command_received = false;
                    motor_stop();
                }

                std::size_t newline = std::string::npos;
                while((newline = command_buffer.find('\n')) != std::string::npos)
                {
                    std::string line = command_buffer.substr(0, newline);
                    command_buffer.erase(0, newline + 1);
                    if(!line.empty() && line.back() == '\r')
                    {
                        line.pop_back();
                    }

                    if(line == "STOP")
                    {
                        command_speed_mps = 0.0;
                        command_steering_deg = config_.servo_center_deg;
                        command_received = true;
                        last_command_time = cycle_start;
                        watchdog_reported = false;
                        resetSpeedLoopState();
                        resetHeadingState();
                        heading_hold_active = false;
                        heading_hold_direction = 0.0;
                        apply_servo_without_delay(command_steering_deg);
                        continue;
                    }

                    std::istringstream command(line);
                    std::string type;
                    double requested_speed = 0.0;
                    double requested_steering = config_.servo_center_deg;
                    if(command >> type >> requested_speed >> requested_steering &&
                       type == "CMD" && std::isfinite(requested_speed) &&
                       std::isfinite(requested_steering))
                    {
                        const double previous_speed = command_speed_mps;
                        command_speed_mps = std::clamp(
                            requested_speed,
                            -max_drive_speed_mps,
                            max_drive_speed_mps);
                        command_steering_deg = std::clamp(
                            requested_steering,
                            std::min(config_.servo_left_deg, config_.servo_right_deg),
                            std::max(config_.servo_left_deg, config_.servo_right_deg));
                        command_received = true;
                        last_command_time = cycle_start;
                        watchdog_reported = false;
                        if(previous_speed * command_speed_mps < 0.0)
                        {
                            resetSpeedLoopState();
                        }
                    }
                }
            }

            const bool command_fresh = enable_remote_drive && command_received &&
                cycle_start - last_command_time <= watchdog_timeout;
            if(enable_remote_drive && command_received && !command_fresh && !watchdog_reported)
            {
                Logger::warn("remote drive watchdog timeout; motors stopped");
                watchdog_reported = true;
                command_speed_mps = 0.0;
                command_steering_deg = config_.servo_center_deg;
                apply_servo_without_delay(command_steering_deg);
                resetSpeedLoopState();
                resetHeadingState();
                heading_hold_active = false;
                heading_hold_direction = 0.0;
            }

            const double drive_direction = command_speed_mps >= 0.0 ? 1.0 : -1.0;
            const bool steering_centered = std::abs(
                command_steering_deg - config_.servo_center_deg) <= 0.25;
            const bool should_hold_heading = command_fresh &&
                std::abs(command_speed_mps) > 0.001 && steering_centered;
            if(should_hold_heading &&
               (!heading_hold_active || drive_direction != heading_hold_direction))
            {
                resetHeadingState();
                heading_hold_direction = drive_direction;
            }
            else if(!should_hold_heading && heading_hold_active)
            {
                resetHeadingState();
                heading_hold_direction = 0.0;
            }
            heading_hold_active = should_hold_heading;
            manual_steering_active_ = !heading_hold_active;
            if(manual_steering_active_)
            {
                apply_servo_without_delay(command_steering_deg);
            }

            int left_count = 0;
            int right_count = 0;
            if(command_fresh && std::abs(command_speed_mps) > 0.001)
            {
                speed_target_l_ = command_speed_mps *
                    config_.left_encoder_counts_per_meter * control_period_seconds;
                speed_target_r_ = command_speed_mps *
                    config_.right_encoder_counts_per_meter * control_period_seconds;
                motor_speed_loop(1, 0.0);
                left_count = encoder_left_;
                right_count = encoder_right_;
            }
            else
            {
                motor_stop();
                const auto counts = readAndClearEncoders();
                left_count = counts.first;
                right_count = counts.second;
            }
            left_odom_accum += left_count;
            right_odom_accum += right_count;

            if(cycle_start >= next_odom_time)
            {
                const double dt_seconds = std::chrono::duration<double>(
                    cycle_start - previous_odom_time).count();
                previous_odom_time = cycle_start;
                next_odom_time = cycle_start + std::chrono::milliseconds(20);

                double gyro_z_dps = 0.0;
#if ROBOT_USE_LS2K0301_LIBRARY
                if(imu_ready_ && g_imu)
                {
                    gyro_z_dps = (static_cast<double>(g_imu->gyroZ()) - imu_gyro_z_bias_) *
                                 config_.imu_gyro_z_scale;
                }
#endif

                std::ostringstream packet;
                packet << "{\"seq\":" << sequence++
                       << ",\"mono_ns\":" << monotonicTimestampNs()
                       << ",\"dt\":" << dt_seconds
                       << ",\"left\":" << left_odom_accum
                       << ",\"right\":" << right_odom_accum
                       << ",\"gyro_z_dps\":" << gyro_z_dps
                       << ",\"imu_ready\":" << (imu_ready_ ? "true" : "false")
                       << ",\"remote_drive\":" << (enable_remote_drive ? "true" : "false")
                       << "}\n";
                left_odom_accum = 0;
                right_odom_accum = 0;
                if(!sendAll(client_fd, packet.str()))
                {
                    client_connected = false;
                }
            }

            if(enable_remote_drive && status_tick++ % 100 == 0)
            {
                std::ostringstream status_log;
                status_log << "mapping_drive command_fresh=" << command_fresh
                           << " speed_mps=" << command_speed_mps
                           << " steering_deg=" << command_steering_deg
                           << " heading_hold=" << heading_hold_active
                           << " heading_error_deg=" << heading_yaw_error_
                           << " heading_correction_deg=" << heading_servo_correction_deg_
                           << " left_pwm=" << speed_pwm_l_
                           << " right_pwm=" << speed_pwm_r_;
                Logger::info(status_log.str());
            }
            std::this_thread::sleep_until(cycle_start + std::chrono::milliseconds(10));
        }

        motor_stop();
        resetSpeedLoopState();
        manual_steering_active_ = false;
        apply_servo_without_delay(config_.servo_center_deg);
        close(client_fd);
        Logger::warn("odometry TCP client disconnected; waiting for reconnect");
    }

    motor_stop();
    manual_steering_active_ = false;
    apply_servo_without_delay(config_.servo_center_deg);
    close(server_fd);
    return signal_guard.stopRequested();
}

void RobotHardware::motor_speed_loop(int big_state, double offset)
{
    car_running_ = big_state == 1;
    offset_ = offset;
    speedControlTick();
}

void RobotHardware::motor_stop()
{
    speed_pwm_l_ = 0;
    speed_pwm_r_ = 0;
    speed_motor_out_l_ = 0.0;
    speed_motor_out_r_ = 0.0;
    speed_real_l_ = 0.0;
    speed_real_r_ = 0.0;
    speed_pid_l_.reset();
    speed_pid_r_.reset();
    stopMotors();
}

void RobotHardware::setSteeringServo(double angle_deg)
{
    if(steering_servo_angle_valid_ &&
       std::abs(steering_servo_angle_deg_ - angle_deg) < 0.2)
    {
        return;
    }

    std::ostringstream oss;
    oss << "steering servo angle=" << angle_deg;
    Logger::info(oss.str());
#if ROBOT_USE_LS2K0301_LIBRARY
    writeSteeringServo(angle_deg);
#else
    steering_servo_angle_deg_ = angle_deg;
    steering_servo_angle_valid_ = true;
#endif
    std::this_thread::sleep_for(std::chrono::milliseconds(std::max(50, config_.servo_settle_ms)));
}

void RobotHardware::writeSteeringServo(double angle_deg)
{
    const double clamped_angle = std::clamp(
        angle_deg,
        std::min(config_.servo_left_deg, config_.servo_right_deg),
        std::max(config_.servo_left_deg, config_.servo_right_deg));
#if ROBOT_USE_LS2K0301_LIBRARY
    if(g_servo)
    {
        g_servo->setAngle(clamped_angle);
    }
#endif
    steering_servo_angle_deg_ = clamped_angle;
    steering_servo_angle_valid_ = true;
}

void RobotHardware::centerSteeringServo()
{
    setSteeringServo(config_.servo_center_deg);
}

void RobotHardware::clearEncoders()
{
#if ROBOT_USE_LS2K0301_LIBRARY
    if(g_encoders)
    {
        g_encoders->clear();
    }
#endif
}

std::pair<int, int> RobotHardware::readAndClearEncoders()
{
#if ROBOT_USE_LS2K0301_LIBRARY
    if(g_encoders)
    {
        return g_encoders->readAndClear();
    }
#endif
    return {0, 0};
}

int RobotHardware::read_encoder_left()
{
#if ROBOT_USE_LS2K0301_LIBRARY
    if(g_encoders)
    {
        const auto [left, right] = g_encoders->readSigned();
        (void)right;
        return left;
    }
#endif
    return 0;
}

int RobotHardware::read_encoder_right()
{
#if ROBOT_USE_LS2K0301_LIBRARY
    if(g_encoders)
    {
        const auto [left, right] = g_encoders->readSigned();
        (void)left;
        return right;
    }
#endif
    return 0;
}

void RobotHardware::set_motor_left_pwm(int pwm)
{
    const int duty = clampPwmDuty(pwm, left_pwm_duty_max_);
#if ROBOT_USE_LS2K0301_LIBRARY
    if(g_motor)
    {
        g_motor->setLeftDuty(duty);
    }
#else
    (void)duty;
#endif
}

void RobotHardware::set_motor_right_pwm(int pwm)
{
    const int duty = clampPwmDuty(pwm, right_pwm_duty_max_);
#if ROBOT_USE_LS2K0301_LIBRARY
    if(g_motor)
    {
        g_motor->setRightDuty(duty);
    }
#else
    (void)duty;
#endif
}

void RobotHardware::set_motor_left_dir(int dir)
{
#if ROBOT_USE_LS2K0301_LIBRARY
    if(g_motor)
    {
        g_motor->setLeftDirection(dir);
    }
#else
    (void)dir;
#endif
}

void RobotHardware::set_motor_right_dir(int dir)
{
#if ROBOT_USE_LS2K0301_LIBRARY
    if(g_motor)
    {
        g_motor->setRightDirection(dir);
    }
#else
    (void)dir;
#endif
}

void RobotHardware::playBeepPattern(int count, int on_ms, int off_ms)
{
    if(!config_.status_beep_enable)
    {
        return;
    }
    count = std::clamp(count, 0, 10);
    on_ms = std::clamp(on_ms, 10, 2000);
    off_ms = std::clamp(off_ms, 0, 2000);
    for(int i = 0; i < count; ++i)
    {
#if ROBOT_USE_LS2K0301_LIBRARY
        if(g_beep)
        {
            g_beep->on();
        }
#endif
        std::this_thread::sleep_for(std::chrono::milliseconds(on_ms));
#if ROBOT_USE_LS2K0301_LIBRARY
        if(g_beep)
        {
            g_beep->off();
        }
#endif
        if(i + 1 < count && off_ms > 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(off_ms));
        }
    }
}

void RobotHardware::beepOff()
{
#if ROBOT_USE_LS2K0301_LIBRARY
    if(g_beep)
    {
        g_beep->off();
    }
#endif
}

void RobotHardware::stopMotors()
{
    car_running_ = false;
#if ROBOT_USE_LS2K0301_LIBRARY
    if(g_motor)
    {
        g_motor->stop();
    }
#endif
}

void RobotHardware::resetSpeedLoopState()
{
    speed_pid_l_.reset();
    speed_pid_r_.reset();
    encoder_left_ = 0;
    encoder_right_ = 0;
    speed_real_l_ = 0.0;
    speed_real_r_ = 0.0;
    speed_pwm_l_ = 0;
    speed_pwm_r_ = 0;
    speed_motor_out_l_ = 0.0;
    speed_motor_out_r_ = 0.0;
    speed_dir_l_ = config_.left_motor_forward_dir;
    speed_dir_r_ = config_.right_motor_forward_dir;
    speed_encoder_count_100ms_l_ = 0;
    speed_encoder_count_100ms_r_ = 0;
    speed_encoder_count_100ms_accum_l_ = 0;
    speed_encoder_count_100ms_accum_r_ = 0;
    speed_loop_100ms_sample_count_ = 0;
}

void RobotHardware::resetHeadingState()
{
    heading_pid_.reset();
    imu_gyro_z_raw_ = 0.0;
    heading_yaw_error_ = 0.0;
    heading_servo_correction_deg_ = 0.0;
    heading_servo_angle_deg_ = config_.servo_center_deg;
}

void RobotHardware::calibrateImuGyroZ(bool force)
{
#if ROBOT_USE_LS2K0301_LIBRARY
    if(!config_.imu_heading_enable || !imu_ready_ || !g_imu)
    {
        return;
    }
    if(imu_gyro_z_calibrated_ && !force)
    {
        Logger::info("using cached imu gyro_z bias=" + std::to_string(imu_gyro_z_bias_));
        return;
    }

    const int samples = std::max(1, config_.imu_calibration_samples);
    long long sum = 0;
    Logger::info("calibrating imu gyro_z bias, keep car still");
    for(int i = 0; i < samples; ++i)
    {
        sum += g_imu->gyroZ();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    imu_gyro_z_bias_ = static_cast<double>(sum) / static_cast<double>(samples);
    imu_gyro_z_calibrated_ = true;

    std::ostringstream oss;
    oss << "imu gyro_z bias=" << imu_gyro_z_bias_ << " samples=" << samples;
    Logger::info(oss.str());
#else
    (void)force;
#endif
}

void RobotHardware::updateHeadingHold(double dt_seconds, double drive_direction)
{
#if ROBOT_USE_LS2K0301_LIBRARY
    if(!config_.imu_heading_enable || !imu_ready_ || !g_imu || !g_servo)
    {
        return;
    }

    imu_gyro_z_raw_ = static_cast<double>(g_imu->gyroZ());
    const double gyro_z = (imu_gyro_z_raw_ - imu_gyro_z_bias_) * config_.imu_gyro_z_scale;
    heading_yaw_error_ += gyro_z * dt_seconds;

    const double correction = heading_pid_.update(0.0, heading_yaw_error_, dt_seconds);
    heading_servo_correction_deg_ = std::clamp(
        correction * config_.heading_servo_dir *
            (drive_direction >= 0.0 ? 1.0 : -1.0),
        -std::abs(config_.heading_output_limit_deg),
        std::abs(config_.heading_output_limit_deg));
    heading_servo_angle_deg_ = std::clamp(
        config_.servo_center_deg + heading_servo_correction_deg_,
        std::min(config_.servo_left_deg, config_.servo_right_deg),
        std::max(config_.servo_left_deg, config_.servo_right_deg));
    writeSteeringServo(heading_servo_angle_deg_);
#else
    (void)dt_seconds;
    (void)drive_direction;
#endif
}

double RobotHardware::pidOutputToMotorOut(double pid_out, double speed_b, double speed_k) const
{
    const double scale_k = std::max(1.0, std::abs(speed_k));
    if(pid_out >= 0.0)
    {
        return (pid_out + speed_b) / scale_k;
    }
    return (pid_out - speed_b) / scale_k;
}

void RobotHardware::speedControlTick()
{
    if(!car_running_)
    {
        motor_stop();
        return;
    }

    constexpr double dt_seconds = 0.01;
    if(!manual_steering_active_)
    {
        const double average_target = 0.5 * (speed_target_l_ + speed_target_r_);
        updateHeadingHold(dt_seconds, average_target >= 0.0 ? 1.0 : -1.0);
    }
    const double left_target_real = speed_target_l_ * (1.0 + speed_dif_p_ * offset_ * 0.01);
    const double right_target_real = speed_target_r_ * (1.0 - speed_dif_p_ * offset_ * 0.01);

    const auto [left_speed, right_speed] = readAndClearEncoders();
    encoder_left_ = left_speed;
    encoder_right_ = right_speed;
    speed_encoder_count_100ms_accum_l_ += encoder_left_;
    speed_encoder_count_100ms_accum_r_ += encoder_right_;
    speed_loop_100ms_sample_count_++;
    if(speed_loop_100ms_sample_count_ >= 10)
    {
        speed_encoder_count_100ms_l_ = speed_encoder_count_100ms_accum_l_;
        speed_encoder_count_100ms_r_ = speed_encoder_count_100ms_accum_r_;
        speed_encoder_count_100ms_accum_l_ = 0;
        speed_encoder_count_100ms_accum_r_ = 0;
        speed_loop_100ms_sample_count_ = 0;
    }

    speed_real_l_ = static_cast<double>(encoder_left_);
    speed_real_r_ = static_cast<double>(encoder_right_);

    if(std::abs(left_target_real) < 0.001)
    {
        speed_pid_l_.reset();
        speed_motor_out_l_ = 0.0;
    }
    else
    {
        const double left_pid_out = speed_pid_l_.update(left_target_real, speed_real_l_, dt_seconds);
        speed_motor_out_l_ = pidOutputToMotorOut(left_pid_out, speed_b_l_, speed_k_l_);
    }

    if(std::abs(right_target_real) < 0.001)
    {
        speed_pid_r_.reset();
        speed_motor_out_r_ = 0.0;
    }
    else
    {
        const double right_pid_out = speed_pid_r_.update(right_target_real, speed_real_r_, dt_seconds);
        speed_motor_out_r_ = pidOutputToMotorOut(right_pid_out, speed_b_r_, speed_k_r_);
    }

    if(left_target_real >= 0.0 && speed_motor_out_l_ < 0.0)
    {
        speed_motor_out_l_ = 0.0;
    }
    else if(left_target_real < 0.0 && speed_motor_out_l_ > 0.0)
    {
        speed_motor_out_l_ = 0.0;
    }

    if(right_target_real >= 0.0 && speed_motor_out_r_ < 0.0)
    {
        speed_motor_out_r_ = 0.0;
    }
    else if(right_target_real < 0.0 && speed_motor_out_r_ > 0.0)
    {
        speed_motor_out_r_ = 0.0;
    }

    speed_pwm_l_ = clampPwmDuty(static_cast<int>(std::abs(speed_motor_out_l_) * 100.0), left_pwm_duty_max_);
    speed_pwm_r_ = clampPwmDuty(static_cast<int>(std::abs(speed_motor_out_r_) * 100.0), right_pwm_duty_max_);
    if(std::abs(left_target_real) >= 0.001 && speed_pwm_l_ > 0)
    {
        speed_pwm_l_ = clampPwmDuty(
            std::max(speed_pwm_l_, config_.left_speed_min_pwm),
            left_pwm_duty_max_);
    }
    if(std::abs(right_target_real) >= 0.001 && speed_pwm_r_ > 0)
    {
        speed_pwm_r_ = clampPwmDuty(
            std::max(speed_pwm_r_, config_.right_speed_min_pwm),
            right_pwm_duty_max_);
    }
    speed_dir_l_ = left_target_real >= 0.0 ? config_.left_motor_forward_dir : inverted(config_.left_motor_forward_dir);
    speed_dir_r_ = right_target_real >= 0.0 ? config_.right_motor_forward_dir : inverted(config_.right_motor_forward_dir);

    set_motor_left_dir(speed_dir_l_);
    set_motor_right_dir(speed_dir_r_);
    set_motor_left_pwm(speed_pwm_l_);
    set_motor_right_pwm(speed_pwm_r_);
}

} // namespace robot
