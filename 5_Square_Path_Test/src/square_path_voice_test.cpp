#include "zf_driver_delay.hpp"
#include "zf_driver_encoder.hpp"
#include "zf_driver_gpio.hpp"
#include "zf_driver_pwm.hpp"
#include "zf_device_imu.hpp"

#include <algorithm>
#include <cerrno>
#include <csignal>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <map>
#include <sstream>
#include <string>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
#include <vector>

namespace {

constexpr const char* kDefaultConfigPath = "/home/root/square_path_config.ini";
constexpr int kFrameIdleMs = 150;
volatile sig_atomic_t g_stop_requested = 0;

enum class VoiceCommand {
    kUnknown,
    kStart,
    kStop,
    kRunSquare,
    kRunSide,
    kTurnLeft,
    kTurnRight,
};

struct Config {
    std::string voice_port = "/dev/ttyS1";
    int baud = 115200;
    int motor1_forward_dir = 0;
    int motor2_forward_dir = 0;
    float servo_center_deg = 105.0f;
    float servo_left_deg = 120.0f;
    float servo_right_deg = 90.0f;
    int servo_settle_ms = 300;
    std::string turn_direction = "right";
    int side_counts = 65;
    int turn_counts_90 = 90;
    int straight_pulse_on_ms = 8;
    int straight_pulse_off_ms = 40;
    int turn_pulse_on_ms = 10;
    int turn_pulse_off_ms = 30;
    int action_timeout_ms = 6000;
    int turn_action_timeout_ms = 10000;
    int max_encoder_delta = 30;
    int corner_pause_ms = 500;
    int square_loops = 1;
    bool require_start = false;
    bool stop_on_any_voice_during_motion = true;
    bool imu_enabled = true;
    float imu_turn_target_deg = 86.0f;
    float imu_gyro_divisor = 16.4f;
    float imu_gyro_deadband_dps = 1.5f;
    int imu_calibration_samples = 80;
    int imu_calibration_delay_ms = 5;
    int imu_sample_ms = 10;
    int imu_min_turn_ms = 300;
    std::map<VoiceCommand, std::vector<unsigned char>> frames;
};

void on_signal(int)
{
    g_stop_requested = 1;
}

long now_ms()
{
    timespec ts {};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<long>(ts.tv_sec) * 1000L + ts.tv_nsec / 1000000L;
}

std::string trim(const std::string& input)
{
    size_t begin = 0;
    while (begin < input.size() && std::isspace(static_cast<unsigned char>(input[begin]))) ++begin;
    size_t end = input.size();
    while (end > begin && std::isspace(static_cast<unsigned char>(input[end - 1]))) --end;
    return input.substr(begin, end - begin);
}

int parse_int(const std::string& text, int fallback)
{
    char* end = nullptr;
    const long value = std::strtol(text.c_str(), &end, 10);
    if (end == text.c_str() || *end != '\0') return fallback;
    return static_cast<int>(value);
}

float parse_float(const std::string& text, float fallback)
{
    char* end = nullptr;
    const float value = std::strtof(text.c_str(), &end);
    if (end == text.c_str() || *end != '\0') return fallback;
    return value;
}

std::string lower_copy(std::string text)
{
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return text;
}

bool parse_bool(const std::string& text, bool fallback)
{
    const std::string value = lower_copy(trim(text));
    if (value == "1" || value == "true" || value == "yes" || value == "on") return true;
    if (value == "0" || value == "false" || value == "no" || value == "off") return false;
    return fallback;
}

std::vector<unsigned char> hex_to_bytes(const std::string& text)
{
    std::string compact;
    for (char c : text) {
        if (!std::isspace(static_cast<unsigned char>(c)) && c != ',' && c != ':' && c != '-') compact.push_back(c);
    }
    if (compact.size() % 2 != 0) return {};

    std::vector<unsigned char> bytes;
    for (size_t i = 0; i < compact.size(); i += 2) {
        char pair[3] = {compact[i], compact[i + 1], '\0'};
        char* end = nullptr;
        const long value = std::strtol(pair, &end, 16);
        if (end == pair || *end != '\0' || value < 0 || value > 255) return {};
        bytes.push_back(static_cast<unsigned char>(value));
    }
    return bytes;
}

std::string bytes_to_hex(const std::vector<unsigned char>& bytes)
{
    static const char* digits = "0123456789ABCDEF";
    std::string out;
    for (unsigned char byte : bytes) {
        if (!out.empty()) out.push_back(' ');
        out.push_back(digits[(byte >> 4) & 0x0F]);
        out.push_back(digits[byte & 0x0F]);
    }
    return out;
}

std::string bytes_to_ascii(const std::vector<unsigned char>& bytes)
{
    std::string out;
    for (unsigned char byte : bytes) {
        out.push_back(std::isprint(byte) ? static_cast<char>(byte) : '.');
    }
    return out;
}

speed_t baud_to_speed(int baud)
{
    switch (baud) {
        case 9600: return B9600;
        case 57600: return B57600;
        case 115200: return B115200;
        default: return B115200;
    }
}

class SerialPort {
public:
    ~SerialPort()
    {
        close_port();
    }

    bool open_port(const std::string& path, int baud)
    {
        close_port();
        fd_ = open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) {
            std::printf("open %s failed: %s\n", path.c_str(), std::strerror(errno));
            return false;
        }

        termios tio {};
        if (tcgetattr(fd_, &tio) != 0) {
            std::printf("tcgetattr failed: %s\n", std::strerror(errno));
            close_port();
            return false;
        }
        cfmakeraw(&tio);
        const speed_t speed = baud_to_speed(baud);
        cfsetispeed(&tio, speed);
        cfsetospeed(&tio, speed);
        tio.c_cflag |= (CLOCAL | CREAD);
        tio.c_cflag &= ~CRTSCTS;
        tio.c_cflag &= ~CSTOPB;
        tio.c_cflag &= ~PARENB;
        tio.c_cflag &= ~CSIZE;
        tio.c_cflag |= CS8;
        tio.c_cc[VMIN] = 0;
        tio.c_cc[VTIME] = 0;
        if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
            std::printf("tcsetattr failed: %s\n", std::strerror(errno));
            close_port();
            return false;
        }
        tcflush(fd_, TCIOFLUSH);
        return true;
    }

    void close_port()
    {
        if (fd_ >= 0) {
            close(fd_);
            fd_ = -1;
        }
    }

    int fd() const
    {
        return fd_;
    }

    bool read_some(std::vector<unsigned char>* out)
    {
        unsigned char buf[128] {};
        const ssize_t n = read(fd_, buf, sizeof(buf));
        if (n > 0) {
            out->insert(out->end(), buf, buf + n);
            return true;
        }
        return false;
    }

private:
    int fd_ = -1;
};

class FrameReader {
public:
    bool poll(SerialPort& serial, int wait_ms, std::vector<unsigned char>* frame)
    {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(serial.fd(), &rfds);
        timeval tv {};
        tv.tv_sec = wait_ms / 1000;
        tv.tv_usec = (wait_ms % 1000) * 1000;
        const int rc = select(serial.fd() + 1, &rfds, nullptr, nullptr, &tv);
        const long t = now_ms();

        if (rc > 0 && FD_ISSET(serial.fd(), &rfds)) {
            std::vector<unsigned char> bytes;
            if (serial.read_some(&bytes) && !bytes.empty()) {
                last_rx_ms_ = t;
                buffer_.insert(buffer_.end(), bytes.begin(), bytes.end());
            }
        }

        if (!buffer_.empty() && (t - last_rx_ms_) >= kFrameIdleMs) {
            *frame = buffer_;
            buffer_.clear();
            last_rx_ms_ = 0;
            return true;
        }
        return false;
    }

private:
    std::vector<unsigned char> buffer_;
    long last_rx_ms_ = 0;
};

VoiceCommand command_from_key(const std::string& key)
{
    if (key == "cmd_start") return VoiceCommand::kStart;
    if (key == "cmd_stop") return VoiceCommand::kStop;
    if (key == "cmd_run_square") return VoiceCommand::kRunSquare;
    if (key == "cmd_run_side") return VoiceCommand::kRunSide;
    if (key == "cmd_turn_left") return VoiceCommand::kTurnLeft;
    if (key == "cmd_turn_right") return VoiceCommand::kTurnRight;
    return VoiceCommand::kUnknown;
}

bool load_config(const std::string& path, Config* config)
{
    FILE* fp = std::fopen(path.c_str(), "r");
    if (!fp) {
        std::printf("config %s not found, using defaults\n", path.c_str());
        return false;
    }
    char line[512] {};
    while (std::fgets(line, sizeof(line), fp)) {
        const std::string text = trim(line);
        if (text.empty() || text[0] == '#') continue;
        const size_t pos = text.find('=');
        if (pos == std::string::npos) continue;
        const std::string key = trim(text.substr(0, pos));
        const std::string value = trim(text.substr(pos + 1));

        if (key == "voice_port") config->voice_port = value;
        else if (key == "baud") config->baud = parse_int(value, config->baud);
        else if (key == "motor1_forward_dir") config->motor1_forward_dir = parse_int(value, config->motor1_forward_dir) ? 1 : 0;
        else if (key == "motor2_forward_dir") config->motor2_forward_dir = parse_int(value, config->motor2_forward_dir) ? 1 : 0;
        else if (key == "servo_center_deg") config->servo_center_deg = parse_float(value, config->servo_center_deg);
        else if (key == "servo_left_deg") config->servo_left_deg = parse_float(value, config->servo_left_deg);
        else if (key == "servo_right_deg") config->servo_right_deg = parse_float(value, config->servo_right_deg);
        else if (key == "servo_settle_ms") config->servo_settle_ms = parse_int(value, config->servo_settle_ms);
        else if (key == "turn_direction") config->turn_direction = lower_copy(value);
        else if (key == "side_counts") config->side_counts = parse_int(value, config->side_counts);
        else if (key == "turn_counts_90") config->turn_counts_90 = parse_int(value, config->turn_counts_90);
        else if (key == "straight_pulse_on_ms") config->straight_pulse_on_ms = parse_int(value, config->straight_pulse_on_ms);
        else if (key == "straight_pulse_off_ms") config->straight_pulse_off_ms = parse_int(value, config->straight_pulse_off_ms);
        else if (key == "turn_pulse_on_ms") config->turn_pulse_on_ms = parse_int(value, config->turn_pulse_on_ms);
        else if (key == "turn_pulse_off_ms") config->turn_pulse_off_ms = parse_int(value, config->turn_pulse_off_ms);
        else if (key == "pulse_on_ms") {
            config->straight_pulse_on_ms = parse_int(value, config->straight_pulse_on_ms);
            config->turn_pulse_on_ms = config->straight_pulse_on_ms;
        }
        else if (key == "pulse_off_ms") {
            config->straight_pulse_off_ms = parse_int(value, config->straight_pulse_off_ms);
            config->turn_pulse_off_ms = config->straight_pulse_off_ms;
        }
        else if (key == "action_timeout_ms") config->action_timeout_ms = parse_int(value, config->action_timeout_ms);
        else if (key == "turn_action_timeout_ms") config->turn_action_timeout_ms = parse_int(value, config->turn_action_timeout_ms);
        else if (key == "max_encoder_delta") config->max_encoder_delta = parse_int(value, config->max_encoder_delta);
        else if (key == "corner_pause_ms") config->corner_pause_ms = parse_int(value, config->corner_pause_ms);
        else if (key == "square_loops") config->square_loops = parse_int(value, config->square_loops);
        else if (key == "require_start") config->require_start = parse_bool(value, config->require_start);
        else if (key == "stop_on_any_voice_during_motion") config->stop_on_any_voice_during_motion = parse_bool(value, config->stop_on_any_voice_during_motion);
        else if (key == "imu_enabled") config->imu_enabled = parse_bool(value, config->imu_enabled);
        else if (key == "imu_turn_target_deg") config->imu_turn_target_deg = parse_float(value, config->imu_turn_target_deg);
        else if (key == "imu_gyro_divisor") config->imu_gyro_divisor = parse_float(value, config->imu_gyro_divisor);
        else if (key == "imu_gyro_deadband_dps") config->imu_gyro_deadband_dps = parse_float(value, config->imu_gyro_deadband_dps);
        else if (key == "imu_calibration_samples") config->imu_calibration_samples = parse_int(value, config->imu_calibration_samples);
        else if (key == "imu_calibration_delay_ms") config->imu_calibration_delay_ms = parse_int(value, config->imu_calibration_delay_ms);
        else if (key == "imu_sample_ms") config->imu_sample_ms = parse_int(value, config->imu_sample_ms);
        else if (key == "imu_min_turn_ms") config->imu_min_turn_ms = parse_int(value, config->imu_min_turn_ms);
        else {
            const VoiceCommand cmd = command_from_key(key);
            if (cmd != VoiceCommand::kUnknown) config->frames[cmd] = hex_to_bytes(value);
        }
    }
    std::fclose(fp);
    return true;
}

bool frame_is(const std::vector<unsigned char>& frame, const char* hex)
{
    return frame == hex_to_bytes(hex);
}

VoiceCommand match_command(const Config& config, const std::vector<unsigned char>& frame)
{
    for (const auto& item : config.frames) {
        if (!item.second.empty() && item.second == frame) return item.first;
    }
    if (frame_is(frame, "AA 55 03 00 FB")) return VoiceCommand::kStart;
    if (frame_is(frame, "AA 55 00 01 FB") || frame_is(frame, "AA 55 00 02 FB")) return VoiceCommand::kStop;
    if (frame_is(frame, "AA 55 00 04 FB")) return VoiceCommand::kRunSquare;
    if (frame_is(frame, "AA 55 00 05 FB")) return VoiceCommand::kRunSide;
    if (frame_is(frame, "AA 55 00 06 FB")) return VoiceCommand::kTurnLeft;
    if (frame_is(frame, "AA 55 00 07 FB")) return VoiceCommand::kTurnRight;

    const std::string ascii = lower_copy(bytes_to_ascii(frame));
    if (ascii.find("start") != std::string::npos || ascii.find("开始") != std::string::npos) return VoiceCommand::kStart;
    if (ascii.find("stop") != std::string::npos || ascii.find("停止") != std::string::npos) return VoiceCommand::kStop;
    if (ascii.find("forward") != std::string::npos || ascii.find("前进") != std::string::npos) return VoiceCommand::kRunSquare;
    if (ascii.find("back") != std::string::npos || ascii.find("后退") != std::string::npos) return VoiceCommand::kRunSide;
    if (ascii.find("left") != std::string::npos || ascii.find("左转") != std::string::npos) return VoiceCommand::kTurnLeft;
    if (ascii.find("right") != std::string::npos || ascii.find("右转") != std::string::npos) return VoiceCommand::kTurnRight;
    return VoiceCommand::kUnknown;
}

const char* command_name(VoiceCommand cmd)
{
    switch (cmd) {
        case VoiceCommand::kStart: return "start";
        case VoiceCommand::kStop: return "stop";
        case VoiceCommand::kRunSquare: return "run_square";
        case VoiceCommand::kRunSide: return "run_side";
        case VoiceCommand::kTurnLeft: return "turn_left";
        case VoiceCommand::kTurnRight: return "turn_right";
        default: return "unknown";
    }
}

class MotorController {
public:
    MotorController()
        : pwm1_(ZF_GPIO_MOTOR_1, O_RDWR),
          pwm2_(ZF_GPIO_MOTOR_2, O_RDWR),
          dir1_(ZF_PWM_MOTOR_1),
          dir2_(ZF_PWM_MOTOR_2),
          enc1_(ZF_ENCODER_QUAD_1),
          enc2_(ZF_ENCODER_QUAD_2)
    {
        dir1_.get_dev_info(&dir1_info_);
        dir2_.get_dev_info(&dir2_info_);
    }

    void stop_all()
    {
        pwm1_.set_level(0);
        pwm2_.set_level(0);
        dir1_.set_duty(0);
        dir2_.set_duty(0);
    }

    void clear_encoders()
    {
        enc1_.clear_count();
        enc2_.clear_count();
    }

    void set_dirs(int motor1_dir, int motor2_dir)
    {
        dir1_.set_duty(static_cast<uint16>(motor1_dir ? dir1_info_.duty_max : 0));
        dir2_.set_duty(static_cast<uint16>(motor2_dir ? dir2_info_.duty_max : 0));
    }

    void pulse(int pulse_on_ms)
    {
        pwm1_.set_level(1);
        pwm2_.set_level(1);
        system_delay_ms(std::max(1, pulse_on_ms));
        pwm1_.set_level(0);
        pwm2_.set_level(0);
    }

    int read_enc1()
    {
        const int value = static_cast<int>(enc1_.get_count());
        enc1_.clear_count();
        return value;
    }

    int read_enc2()
    {
        const int value = static_cast<int>(enc2_.get_count());
        enc2_.clear_count();
        return value;
    }

private:
    zf_driver_gpio pwm1_;
    zf_driver_gpio pwm2_;
    zf_driver_pwm dir1_;
    zf_driver_pwm dir2_;
    zf_driver_encoder enc1_;
    zf_driver_encoder enc2_;
    pwm_info dir1_info_ {};
    pwm_info dir2_info_ {};
};

class SteeringServo {
public:
    SteeringServo()
        : servo_(ZF_PWM_SERVO_1)
    {
        servo_.get_dev_info(&info_);
    }

    void set_angle(float angle_deg, int hold_ms)
    {
        const uint16 duty = duty_from_angle(angle_deg);
        std::printf("servo angle=%.1f duty=%u hold=%dms\n", angle_deg, duty, hold_ms);
        servo_.set_duty(duty);
        system_delay_ms(std::max(50, hold_ms));
    }

    void center(const Config& config)
    {
        set_angle(config.servo_center_deg, config.servo_settle_ms);
    }

    void release()
    {
        servo_.set_duty(0);
    }

private:
    uint16 duty_from_angle(float angle_deg) const
    {
        angle_deg = std::clamp(angle_deg, 45.0f, 135.0f);
        if (info_.freq == 0 || info_.duty_max == 0) return 0;
        const float pulse_ms = 0.5f + angle_deg / 90.0f;
        const float period_ms = 1000.0f / static_cast<float>(info_.freq);
        const float duty = static_cast<float>(info_.duty_max) * pulse_ms / period_ms;
        return static_cast<uint16>(std::clamp(duty, 0.0f, static_cast<float>(info_.duty_max)));
    }

    zf_driver_pwm servo_;
    pwm_info info_ {};
};

class ImuYawEstimator {
public:
    bool init(const Config& config)
    {
        if (!config.imu_enabled) {
            std::puts("IMU disabled by config; turns will use encoder fallback.");
            return false;
        }

        const imu_device_type_enum type = imu_.init();
        if (type == DEV_NO_FIND) {
            std::puts("IMU not found; turns will use encoder fallback.");
            return false;
        }

        divisor_ = config.imu_gyro_divisor > 0.1f ? config.imu_gyro_divisor : default_divisor(type);
        deadband_dps_ = std::max(0.0f, config.imu_gyro_deadband_dps);
        available_ = true;
        std::printf("IMU ready: type=%d gyro_divisor=%.3f deadband=%.2f dps\n",
                    static_cast<int>(type), divisor_, deadband_dps_);
        calibrate(config);
        return true;
    }

    bool available() const
    {
        return available_;
    }

    void calibrate(const Config& config)
    {
        if (!available_) return;
        const int samples = std::clamp(config.imu_calibration_samples, 1, 500);
        const int delay_ms = std::clamp(config.imu_calibration_delay_ms, 1, 50);
        long total = 0;
        for (int i = 0; i < samples; ++i) {
            total += imu_.get_gyro_z();
            system_delay_ms(delay_ms);
        }
        bias_raw_ = static_cast<float>(total) / static_cast<float>(samples);
        std::printf("IMU gyro_z bias raw=%.2f samples=%d\n", bias_raw_, samples);
    }

    void begin_turn()
    {
        yaw_deg_ = 0.0f;
        last_ms_ = now_ms();
    }

    float update()
    {
        if (!available_) return yaw_deg_;
        const long t = now_ms();
        long dt_ms = t - last_ms_;
        if (dt_ms < 0) dt_ms = 0;
        if (dt_ms > 200) dt_ms = 200;
        last_ms_ = t;

        const int raw = imu_.get_gyro_z();
        last_raw_ = raw;
        float dps = (static_cast<float>(raw) - bias_raw_) / divisor_;
        if (std::fabs(dps) < deadband_dps_) dps = 0.0f;
        last_dps_ = dps;
        yaw_deg_ += dps * (static_cast<float>(dt_ms) / 1000.0f);
        return yaw_deg_;
    }

    int last_raw() const
    {
        return last_raw_;
    }

    float last_dps() const
    {
        return last_dps_;
    }

    float yaw_deg() const
    {
        return yaw_deg_;
    }

private:
    static float default_divisor(imu_device_type_enum type)
    {
        if (type == DEV_IMU660RB || type == DEV_IMU963RA) return 14.3f;
        return 16.4f;
    }

    zf_device_imu imu_;
    bool available_ = false;
    float divisor_ = 16.4f;
    float deadband_dps_ = 1.5f;
    float bias_raw_ = 0.0f;
    float yaw_deg_ = 0.0f;
    float last_dps_ = 0.0f;
    int last_raw_ = 0;
    long last_ms_ = 0;
};

MotorController* g_motor = nullptr;
SteeringServo* g_servo = nullptr;

bool wait_with_stop_poll(SerialPort& serial, FrameReader& reader, const Config& config, int wait_ms)
{
    if (serial.fd() < 0) {
        system_delay_ms(std::max(1, wait_ms));
        return !g_stop_requested;
    }
    const long deadline = now_ms() + wait_ms;
    while (!g_stop_requested && now_ms() < deadline) {
        std::vector<unsigned char> frame;
        if (reader.poll(serial, 25, &frame)) {
            const VoiceCommand cmd = match_command(config, frame);
            std::printf("voice during motion hex=[%s] command=%s\n", bytes_to_hex(frame).c_str(), command_name(cmd));
            if (cmd == VoiceCommand::kStop || config.stop_on_any_voice_during_motion) {
                std::printf("motion stopped by voice frame command=%s\n", command_name(cmd));
                return false;
            }
        }
    }
    return !g_stop_requested;
}

bool drive_counts(MotorController& motor, SerialPort& serial, FrameReader& reader,
                  const Config& config, int target_counts, int motor1_dir, int motor2_dir,
                  int pulse_on_ms, int pulse_off_ms, int timeout_ms)
{
    target_counts = std::max(1, target_counts);
    motor.stop_all();
    system_delay_ms(100);
    motor.clear_encoders();
    motor.set_dirs(motor1_dir, motor2_dir);

    int enc1_total = 0;
    int enc2_total = 0;
    int sample = 0;
    const long deadline = now_ms() + std::max(200, timeout_ms);

    while (!g_stop_requested && now_ms() < deadline) {
        motor.pulse(pulse_on_ms);
        if (!wait_with_stop_poll(serial, reader, config, pulse_off_ms)) {
            motor.stop_all();
            return false;
        }

        const int enc1_delta = motor.read_enc1();
        const int enc2_delta = motor.read_enc2();
        enc1_total += enc1_delta;
        enc2_total += enc2_delta;
        std::printf("motion,%d,%d,%d,%d,%d\n", sample++, enc1_delta, enc2_delta, enc1_total, enc2_total);
        std::fflush(stdout);

        if (std::abs(enc1_delta) > config.max_encoder_delta || std::abs(enc2_delta) > config.max_encoder_delta) {
            std::puts("large encoder jump, stopping");
            break;
        }
        if (std::max(std::abs(enc1_total), std::abs(enc2_total)) >= target_counts) {
            std::puts("target reached");
            break;
        }
    }

    motor.stop_all();
    motor.clear_encoders();
    std::printf("drive end enc=(%d,%d)\n", enc1_total, enc2_total);
    return !g_stop_requested;
}

bool poll_stop_command(SerialPort& serial, FrameReader& reader, const Config& config, int wait_ms)
{
    if (serial.fd() < 0) {
        system_delay_ms(std::max(1, wait_ms));
        return !g_stop_requested;
    }
    std::vector<unsigned char> frame;
    if (!reader.poll(serial, wait_ms, &frame)) return true;
    const VoiceCommand cmd = match_command(config, frame);
    std::printf("voice during motion hex=[%s] command=%s\n", bytes_to_hex(frame).c_str(), command_name(cmd));
    if (cmd == VoiceCommand::kStop || config.stop_on_any_voice_during_motion) {
        std::printf("motion stopped by voice frame command=%s\n", command_name(cmd));
        return false;
    }
    return true;
}

bool drive_yaw(MotorController& motor, ImuYawEstimator& imu, SerialPort& serial, FrameReader& reader,
               const Config& config, float target_deg, int motor1_dir, int motor2_dir,
               int pulse_on_ms, int pulse_off_ms, int timeout_ms)
{
    target_deg = std::clamp(std::fabs(target_deg), 5.0f, 180.0f);
    motor.stop_all();
    system_delay_ms(150);
    motor.clear_encoders();
    imu.calibrate(config);
    imu.begin_turn();
    motor.set_dirs(motor1_dir, motor2_dir);

    int enc1_total = 0;
    int enc2_total = 0;
    int sample = 0;
    long last_print_ms = 0;
    const long start_ms = now_ms();
    const long deadline = start_ms + std::max(200, timeout_ms);
    const int sample_ms = std::clamp(config.imu_sample_ms, 5, 50);

    while (!g_stop_requested && now_ms() < deadline) {
        motor.pulse(pulse_on_ms);
        float yaw = imu.update();

        const long off_deadline = now_ms() + std::max(1, pulse_off_ms);
        while (!g_stop_requested && now_ms() < off_deadline) {
            if (!poll_stop_command(serial, reader, config, sample_ms)) {
                motor.stop_all();
                return false;
            }
            yaw = imu.update();
            const long elapsed_ms = now_ms() - start_ms;
            if (elapsed_ms >= config.imu_min_turn_ms && std::fabs(yaw) >= target_deg) {
                std::printf("IMU yaw target reached: yaw=%.2f deg target=%.2f deg\n", yaw, target_deg);
                motor.stop_all();
                motor.clear_encoders();
                return !g_stop_requested;
            }
        }

        const int enc1_delta = motor.read_enc1();
        const int enc2_delta = motor.read_enc2();
        enc1_total += enc1_delta;
        enc2_total += enc2_delta;

        const long t = now_ms();
        if (t - last_print_ms >= 100) {
            last_print_ms = t;
            std::printf("turn_imu,%d,raw=%d,dps=%.2f,yaw=%.2f,enc=%d,%d\n",
                        sample++, imu.last_raw(), imu.last_dps(), imu.yaw_deg(), enc1_total, enc2_total);
            std::fflush(stdout);
        }

        if (std::abs(enc1_delta) > config.max_encoder_delta || std::abs(enc2_delta) > config.max_encoder_delta) {
            std::puts("large encoder jump, stopping");
            break;
        }
        if (std::max(std::abs(enc1_total), std::abs(enc2_total)) >= config.turn_counts_90 * 2) {
            std::puts("encoder safety limit reached during IMU turn");
            break;
        }
    }

    motor.stop_all();
    motor.clear_encoders();
    std::printf("turn end yaw=%.2f enc=(%d,%d)\n", imu.yaw_deg(), enc1_total, enc2_total);
    return !g_stop_requested;
}

bool run_one_side(MotorController& motor, SteeringServo& servo, SerialPort& serial, FrameReader& reader, const Config& config)
{
    std::printf("single side: counts=%d pulse=%d/%dms\n",
                config.side_counts, config.straight_pulse_on_ms, config.straight_pulse_off_ms);
    servo.center(config);
    return drive_counts(motor, serial, reader, config, config.side_counts,
                        config.motor1_forward_dir, config.motor2_forward_dir,
                        config.straight_pulse_on_ms, config.straight_pulse_off_ms,
                        config.action_timeout_ms);
}

bool run_one_corner(MotorController& motor, SteeringServo& servo, SerialPort& serial, FrameReader& reader,
                    const Config& config, ImuYawEstimator* imu, bool turn_left)
{
    const float corner_angle = turn_left ? config.servo_left_deg : config.servo_right_deg;
    std::printf("single corner: target=%.1fdeg fallback_counts=%d turn=%s servo=%.1f pulse=%d/%dms imu=%s\n",
                config.imu_turn_target_deg, config.turn_counts_90, turn_left ? "left" : "right", corner_angle,
                config.turn_pulse_on_ms, config.turn_pulse_off_ms,
                (imu && imu->available()) ? "on" : "off");
    servo.set_angle(corner_angle, config.servo_settle_ms);
    bool ok = false;
    if (imu && imu->available()) {
        ok = drive_yaw(motor, *imu, serial, reader, config, config.imu_turn_target_deg,
                       config.motor1_forward_dir, config.motor2_forward_dir,
                       config.turn_pulse_on_ms, config.turn_pulse_off_ms,
                       config.turn_action_timeout_ms);
    } else {
        ok = drive_counts(motor, serial, reader, config, config.turn_counts_90,
                          config.motor1_forward_dir, config.motor2_forward_dir,
                          config.turn_pulse_on_ms, config.turn_pulse_off_ms,
                          config.turn_action_timeout_ms);
    }
    servo.center(config);
    return ok;
}

bool run_square(MotorController& motor, SteeringServo& servo, SerialPort& serial, FrameReader& reader,
                const Config& config, ImuYawEstimator* imu)
{
    const bool turn_left = config.turn_direction == "left";
    const int loops = std::clamp(config.square_loops, 1, 3);
    std::printf("square begin: loops=%d side_counts=%d turn_target=%.1fdeg fallback_counts=%d turn=%s pulse=%d/%dms imu=%s\n",
                loops,
                config.side_counts, config.imu_turn_target_deg, config.turn_counts_90,
                turn_left ? "left" : "right", config.turn_pulse_on_ms, config.turn_pulse_off_ms,
                (imu && imu->available()) ? "on" : "off");

    for (int loop = 1; loop <= loops && !g_stop_requested; ++loop) {
        std::printf("square loop %d/%d\n", loop, loops);
        for (int side = 1; side <= 4 && !g_stop_requested; ++side) {
            std::printf("square side %d: straight\n", side);
            if (!run_one_side(motor, servo, serial, reader, config)) {
                return false;
            }
            system_delay_ms(std::max(0, config.corner_pause_ms));

            std::printf("square corner %d: IMU-assisted turn\n", side);
            if (!run_one_corner(motor, servo, serial, reader, config, imu, turn_left)) {
                return false;
            }
            system_delay_ms(std::max(0, config.corner_pause_ms));
        }
    }

    motor.stop_all();
    servo.center(config);
    std::puts("square done");
    return true;
}

std::string get_arg(int argc, char** argv, const std::string& name, const std::string& fallback)
{
    for (int i = 2; i + 1 < argc; ++i) {
        if (argv[i] == name) return argv[i + 1];
    }
    return fallback;
}

int run_control(int argc, char** argv)
{
    Config config;
    const std::string config_path = get_arg(argc, argv, "--config", kDefaultConfigPath);
    load_config(config_path, &config);

    SerialPort serial;
    if (!serial.open_port(config.voice_port, config.baud)) return 1;

    MotorController motor;
    SteeringServo servo;
    ImuYawEstimator imu;
    g_motor = &motor;
    g_servo = &servo;
    motor.stop_all();
    motor.clear_encoders();
    servo.center(config);
    imu.init(config);

    FrameReader reader;
    bool armed = false;
    std::printf("square voice test ready: port=%s baud=%d require_start=%d state=%s\n",
                config.voice_port.c_str(), config.baud, config.require_start ? 1 : 0,
                config.require_start ? "DISARMED" : "READY");
    std::puts("commands: forward=one square, back=one side, left/right=one corner, stop=stop.");
    std::fflush(stdout);

    while (!g_stop_requested) {
        std::vector<unsigned char> frame;
        if (!reader.poll(serial, 100, &frame)) continue;
        const VoiceCommand cmd = match_command(config, frame);
        std::printf("voice frame hex=[%s] ascii=[%s] command=%s state=%s\n",
                    bytes_to_hex(frame).c_str(), bytes_to_ascii(frame).c_str(), command_name(cmd),
                    armed ? "ARMED" : "DISARMED");
        std::fflush(stdout);

        if (cmd == VoiceCommand::kStart) {
            armed = true;
            std::puts("state=ARMED");
        } else if (cmd == VoiceCommand::kStop) {
            motor.stop_all();
            servo.center(config);
            armed = false;
            std::puts("state=DISARMED");
        } else if ((!config.require_start || armed) && cmd == VoiceCommand::kRunSquare) {
            run_square(motor, servo, serial, reader, config, &imu);
            armed = false;
            std::printf("square command complete, state=%s\n", config.require_start ? "DISARMED" : "READY");
        } else if ((!config.require_start || armed) && cmd == VoiceCommand::kRunSide) {
            run_one_side(motor, servo, serial, reader, config);
            armed = false;
            std::printf("single side complete, state=%s\n", config.require_start ? "DISARMED" : "READY");
        } else if ((!config.require_start || armed) && cmd == VoiceCommand::kTurnLeft) {
            run_one_corner(motor, servo, serial, reader, config, &imu, true);
            armed = false;
            std::printf("single left corner complete, state=%s\n", config.require_start ? "DISARMED" : "READY");
        } else if ((!config.require_start || armed) && cmd == VoiceCommand::kTurnRight) {
            run_one_corner(motor, servo, serial, reader, config, &imu, false);
            armed = false;
            std::printf("single right corner complete, state=%s\n", config.require_start ? "DISARMED" : "READY");
        } else {
            std::puts("ignored");
        }
    }

    motor.stop_all();
    servo.center(config);
    servo.release();
    return 0;
}

int run_imu_probe(int argc, char** argv)
{
    Config config;
    const std::string config_path = get_arg(argc, argv, "--config", kDefaultConfigPath);
    load_config(config_path, &config);
    const int seconds = std::clamp(parse_int(get_arg(argc, argv, "--seconds", "10"), 10), 1, 120);

    ImuYawEstimator imu;
    if (!imu.init(config)) return 1;
    imu.begin_turn();
    const long deadline = now_ms() + seconds * 1000L;
    std::printf("IMU probe running for %ds. Keep car still first, then rotate by hand to verify yaw sign/value.\n", seconds);
    while (!g_stop_requested && now_ms() < deadline) {
        imu.update();
        std::printf("imu raw_z=%d dps_z=%.2f yaw=%.2f\n", imu.last_raw(), imu.last_dps(), imu.yaw_deg());
        std::fflush(stdout);
        system_delay_ms(100);
    }
    return 0;
}

int run_voice_probe(int argc, char** argv)
{
    Config config;
    const std::string config_path = get_arg(argc, argv, "--config", kDefaultConfigPath);
    load_config(config_path, &config);
    const int seconds = std::clamp(parse_int(get_arg(argc, argv, "--seconds", "20"), 20), 1, 300);

    SerialPort serial;
    if (!serial.open_port(config.voice_port, config.baud)) return 1;

    FrameReader reader;
    const long deadline = now_ms() + seconds * 1000L;
    std::printf("voice probe running for %ds on %s baud=%d. Say commands now.\n",
                seconds, config.voice_port.c_str(), config.baud);
    while (!g_stop_requested && now_ms() < deadline) {
        std::vector<unsigned char> frame;
        if (!reader.poll(serial, 100, &frame)) continue;
        const VoiceCommand cmd = match_command(config, frame);
        std::printf("probe voice hex=[%s] ascii=[%s] command=%s\n",
                    bytes_to_hex(frame).c_str(), bytes_to_ascii(frame).c_str(), command_name(cmd));
        std::fflush(stdout);
    }
    return 0;
}

int run_direct_motion(int argc, char** argv, const std::string& mode)
{
    Config config;
    const std::string config_path = get_arg(argc, argv, "--config", kDefaultConfigPath);
    load_config(config_path, &config);

    SerialPort serial;
    FrameReader reader;
    if (!serial.open_port(config.voice_port, config.baud)) {
        std::puts("voice port unavailable; direct motion will run without voice stop polling");
    }

    MotorController motor;
    SteeringServo servo;
    ImuYawEstimator imu;
    g_motor = &motor;
    g_servo = &servo;
    motor.stop_all();
    motor.clear_encoders();
    servo.center(config);
    imu.init(config);

    bool ok = false;
    if (mode == "side") {
        ok = run_one_side(motor, servo, serial, reader, config);
    } else if (mode == "turn-left" || mode == "turn_left") {
        ok = run_one_corner(motor, servo, serial, reader, config, &imu, true);
    } else if (mode == "turn-right" || mode == "turn_right") {
        ok = run_one_corner(motor, servo, serial, reader, config, &imu, false);
    } else if (mode == "square") {
        ok = run_square(motor, servo, serial, reader, config, &imu);
    } else {
        std::printf("unknown direct mode: %s\n", mode.c_str());
        ok = false;
    }

    motor.stop_all();
    servo.center(config);
    servo.release();
    std::printf("direct mode %s %s\n", mode.c_str(), ok ? "done" : "stopped/failed");
    return ok ? 0 : 1;
}

void print_usage(const char* argv0)
{
    std::printf("usage:\n");
    std::printf("  %s run [--config /home/root/square_path_config.ini]\n", argv0);
    std::printf("  %s probe [--config /home/root/square_path_config.ini] [--seconds 20]\n", argv0);
    std::printf("  %s imu [--config /home/root/square_path_config.ini] [--seconds 10]\n", argv0);
    std::printf("  %s side|turn-left|turn-right|square [--config /home/root/square_path_config.ini]\n", argv0);
}

}  // namespace

int main(int argc, char** argv)
{
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
    if (argc < 2) {
        print_usage(argv[0]);
        return 2;
    }
    const std::string mode = argv[1];
    int rc = 2;
    if (mode == "run") {
        rc = run_control(argc, argv);
    } else if (mode == "probe") {
        rc = run_voice_probe(argc, argv);
    } else if (mode == "imu") {
        rc = run_imu_probe(argc, argv);
    } else if (mode == "side" || mode == "turn-left" || mode == "turn_left" ||
               mode == "turn-right" || mode == "turn_right" || mode == "square") {
        rc = run_direct_motion(argc, argv, mode);
    } else {
        print_usage(argv[0]);
    }
    if (g_motor) g_motor->stop_all();
    if (g_servo) g_servo->release();
    return rc;
}
