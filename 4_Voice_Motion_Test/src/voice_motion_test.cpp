#include "zf_driver_encoder.hpp"
#include "zf_driver_gpio.hpp"
#include "zf_driver_pwm.hpp"
#include "zf_driver_delay.hpp"

#include <algorithm>
#include <arpa/inet.h>
#include <cerrno>
#include <csignal>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <functional>
#include <map>
#include <netinet/in.h>
#include <sstream>
#include <string>
#include <sys/socket.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
#include <vector>

namespace {

constexpr const char* kDefaultConfigPath = "/home/root/voice_motion_config.ini";
constexpr int kFrameIdleMs = 150;

volatile sig_atomic_t g_stop_requested = 0;

enum class Command {
    kUnknown,
    kStart,
    kStop,
    kForward,
    kBack,
    kLeft,
    kRight,
};

struct Config {
    std::string voice_port = "/dev/ttyS1";
    int baud = 115200;
    int forward_counts = 25;
    int turn_counts = 15;
    int pulse_on_ms = 1;
    int pulse_off_ms = 300;
    int action_timeout_ms = 8000;
    int max_encoder_delta = 20;
    int motor1_forward_dir = 1;
    int motor2_forward_dir = 1;
    float servo_center_deg = 105.0f;
    float servo_left_deg = 120.0f;
    float servo_right_deg = 90.0f;
    int servo_settle_ms = 300;
    int agent_bind_port = 15000;
    int agent_status_period_ms = 1000;
    std::map<Command, std::vector<unsigned char>> command_frames;
};

struct MotionOutcome {
    int enc1_total = 0;
    int enc2_total = 0;
    bool target_reached = false;
    bool stopped = false;
};

std::string command_name(Command cmd)
{
    switch (cmd) {
        case Command::kStart: return "start";
        case Command::kStop: return "stop";
        case Command::kForward: return "forward";
        case Command::kBack: return "back";
        case Command::kLeft: return "left";
        case Command::kRight: return "right";
        default: return "unknown";
    }
}

Command command_from_name(const std::string& name)
{
    if (name == "start") return Command::kStart;
    if (name == "stop") return Command::kStop;
    if (name == "forward") return Command::kForward;
    if (name == "back") return Command::kBack;
    if (name == "left") return Command::kLeft;
    if (name == "right") return Command::kRight;
    return Command::kUnknown;
}

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
    while (begin < input.size() && std::isspace(static_cast<unsigned char>(input[begin]))) {
        ++begin;
    }
    size_t end = input.size();
    while (end > begin && std::isspace(static_cast<unsigned char>(input[end - 1]))) {
        --end;
    }
    return input.substr(begin, end - begin);
}

int parse_int(const std::string& text, int fallback)
{
    char* end = nullptr;
    const long value = std::strtol(text.c_str(), &end, 10);
    if (end == text.c_str() || *end != '\0') {
        return fallback;
    }
    return static_cast<int>(value);
}

float parse_float(const std::string& text, float fallback)
{
    char* end = nullptr;
    const float value = std::strtof(text.c_str(), &end);
    if (end == text.c_str() || *end != '\0') {
        return fallback;
    }
    return value;
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

std::vector<unsigned char> hex_to_bytes(const std::string& text)
{
    std::string compact;
    for (char c : text) {
        if (!std::isspace(static_cast<unsigned char>(c)) && c != ',' && c != ':' && c != '-') {
            compact.push_back(c);
        }
    }
    if (compact.size() % 2 != 0) {
        return {};
    }

    std::vector<unsigned char> bytes;
    for (size_t i = 0; i < compact.size(); i += 2) {
        char pair[3] = {compact[i], compact[i + 1], '\0'};
        char* end = nullptr;
        const long value = std::strtol(pair, &end, 16);
        if (end == pair || *end != '\0' || value < 0 || value > 255) {
            return {};
        }
        bytes.push_back(static_cast<unsigned char>(value));
    }
    return bytes;
}

bool frame_is(const std::vector<unsigned char>& frame, const char* hex)
{
    return frame == hex_to_bytes(hex);
}

std::string bytes_to_ascii(const std::vector<unsigned char>& bytes)
{
    std::string out;
    for (unsigned char byte : bytes) {
        if (byte == '\r') {
            out += "\\r";
        } else if (byte == '\n') {
            out += "\\n";
        } else if (std::isprint(byte)) {
            out.push_back(static_cast<char>(byte));
        } else {
            out.push_back('.');
        }
    }
    return out;
}

speed_t baud_to_speed(int baud)
{
    switch (baud) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
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
        if (out == nullptr || fd_ < 0) return false;
        unsigned char buf[128] {};
        const ssize_t n = read(fd_, buf, sizeof(buf));
        if (n > 0) {
            out->insert(out->end(), buf, buf + n);
            return true;
        }
        if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            std::printf("serial read failed: %s\n", std::strerror(errno));
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
        if (frame == nullptr || serial.fd() < 0) return false;

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
                if (buffer_.empty()) {
                    first_rx_ms_ = t;
                }
                last_rx_ms_ = t;
                buffer_.insert(buffer_.end(), bytes.begin(), bytes.end());
            }
        }

        if (!buffer_.empty() && (t - last_rx_ms_) >= kFrameIdleMs) {
            *frame = buffer_;
            buffer_.clear();
            first_rx_ms_ = 0;
            last_rx_ms_ = 0;
            return true;
        }
        return false;
    }

    bool wait_frame(SerialPort& serial, int total_timeout_ms, std::vector<unsigned char>* frame)
    {
        const long deadline = now_ms() + total_timeout_ms;
        while (!g_stop_requested && now_ms() < deadline) {
            if (poll(serial, 50, frame)) {
                return true;
            }
        }
        return false;
    }

private:
    std::vector<unsigned char> buffer_;
    long first_rx_ms_ = 0;
    long last_rx_ms_ = 0;
};

bool load_config(const std::string& path, Config* config)
{
    if (config == nullptr) return false;
    FILE* fp = std::fopen(path.c_str(), "r");
    if (fp == nullptr) {
        std::printf("config %s not found, using defaults\n", path.c_str());
        return false;
    }

    char line[512] {};
    while (std::fgets(line, sizeof(line), fp) != nullptr) {
        std::string text = trim(line);
        if (text.empty() || text[0] == '#') continue;
        const size_t pos = text.find('=');
        if (pos == std::string::npos) continue;
        const std::string key = trim(text.substr(0, pos));
        const std::string value = trim(text.substr(pos + 1));

        if (key == "voice_port") config->voice_port = value;
        else if (key == "baud") config->baud = parse_int(value, config->baud);
        else if (key == "forward_counts") config->forward_counts = parse_int(value, config->forward_counts);
        else if (key == "turn_counts") config->turn_counts = parse_int(value, config->turn_counts);
        else if (key == "pulse_on_ms") config->pulse_on_ms = parse_int(value, config->pulse_on_ms);
        else if (key == "pulse_off_ms") config->pulse_off_ms = parse_int(value, config->pulse_off_ms);
        else if (key == "action_timeout_ms") config->action_timeout_ms = parse_int(value, config->action_timeout_ms);
        else if (key == "max_encoder_delta") config->max_encoder_delta = parse_int(value, config->max_encoder_delta);
        else if (key == "motor1_forward_dir") config->motor1_forward_dir = parse_int(value, config->motor1_forward_dir) ? 1 : 0;
        else if (key == "motor2_forward_dir") config->motor2_forward_dir = parse_int(value, config->motor2_forward_dir) ? 1 : 0;
        else if (key == "servo_center_deg") config->servo_center_deg = parse_float(value, config->servo_center_deg);
        else if (key == "servo_left_deg") config->servo_left_deg = parse_float(value, config->servo_left_deg);
        else if (key == "servo_right_deg") config->servo_right_deg = parse_float(value, config->servo_right_deg);
        else if (key == "servo_settle_ms") config->servo_settle_ms = parse_int(value, config->servo_settle_ms);
        else if (key == "agent_bind_port") config->agent_bind_port = parse_int(value, config->agent_bind_port);
        else if (key == "agent_status_period_ms") config->agent_status_period_ms = parse_int(value, config->agent_status_period_ms);
        else if (key.rfind("cmd_", 0) == 0) {
            const Command cmd = command_from_name(key.substr(4));
            if (cmd != Command::kUnknown) {
                config->command_frames[cmd] = hex_to_bytes(value);
            }
        }
    }
    std::fclose(fp);
    return true;
}

bool save_config(const std::string& path, const Config& config)
{
    FILE* fp = std::fopen(path.c_str(), "w");
    if (fp == nullptr) {
        std::printf("write config %s failed: %s\n", path.c_str(), std::strerror(errno));
        return false;
    }
    std::fprintf(fp, "# Library Patrol voice motion test config\n");
    std::fprintf(fp, "voice_port=%s\n", config.voice_port.c_str());
    std::fprintf(fp, "baud=%d\n", config.baud);
    std::fprintf(fp, "forward_counts=%d\n", config.forward_counts);
    std::fprintf(fp, "turn_counts=%d\n", config.turn_counts);
    std::fprintf(fp, "pulse_on_ms=%d\n", config.pulse_on_ms);
    std::fprintf(fp, "pulse_off_ms=%d\n", config.pulse_off_ms);
    std::fprintf(fp, "action_timeout_ms=%d\n", config.action_timeout_ms);
    std::fprintf(fp, "max_encoder_delta=%d\n", config.max_encoder_delta);
    std::fprintf(fp, "motor1_forward_dir=%d\n", config.motor1_forward_dir);
    std::fprintf(fp, "motor2_forward_dir=%d\n", config.motor2_forward_dir);
    std::fprintf(fp, "servo_center_deg=%.1f\n", config.servo_center_deg);
    std::fprintf(fp, "servo_left_deg=%.1f\n", config.servo_left_deg);
    std::fprintf(fp, "servo_right_deg=%.1f\n", config.servo_right_deg);
    std::fprintf(fp, "servo_settle_ms=%d\n", config.servo_settle_ms);
    std::fprintf(fp, "agent_bind_port=%d\n", config.agent_bind_port);
    std::fprintf(fp, "agent_status_period_ms=%d\n", config.agent_status_period_ms);
    for (Command cmd : {Command::kStart, Command::kStop, Command::kForward, Command::kBack, Command::kLeft, Command::kRight}) {
        const auto it = config.command_frames.find(cmd);
        std::fprintf(fp, "cmd_%s=%s\n", command_name(cmd).c_str(),
                     it == config.command_frames.end() ? "" : bytes_to_hex(it->second).c_str());
    }
    std::fclose(fp);
    return true;
}

Command command_from_text(const std::string& text)
{
    if (text.find("start") != std::string::npos || text.find("START") != std::string::npos ||
        text.find("开始控制") != std::string::npos || text.find("解锁") != std::string::npos) {
        return Command::kStart;
    }
    if (text.find("stop") != std::string::npos || text.find("STOP") != std::string::npos ||
        text.find("停止") != std::string::npos || text.find("停车") != std::string::npos) {
        return Command::kStop;
    }
    if (text.find("forward") != std::string::npos || text.find("FORWARD") != std::string::npos ||
        text.find("前进") != std::string::npos) {
        return Command::kForward;
    }
    if (text.find("back") != std::string::npos || text.find("BACK") != std::string::npos ||
        text.find("后退") != std::string::npos) {
        return Command::kBack;
    }
    if (text.find("left") != std::string::npos || text.find("LEFT") != std::string::npos ||
        text.find("左转") != std::string::npos) {
        return Command::kLeft;
    }
    if (text.find("right") != std::string::npos || text.find("RIGHT") != std::string::npos ||
        text.find("右转") != std::string::npos) {
        return Command::kRight;
    }
    return Command::kUnknown;
}

Command match_command(const Config& config, const std::vector<unsigned char>& frame)
{
    for (const auto& item : config.command_frames) {
        if (!item.second.empty() && item.second == frame) {
            return item.first;
        }
    }

    // Yahboom voice module defaults. These make the test usable before
    // custom command learning/firmware generation.
    if (frame_is(frame, "AA 55 03 00 FB")) return Command::kStart;
    if (frame_is(frame, "AA 55 00 01 FB") || frame_is(frame, "AA 55 00 02 FB")) return Command::kStop;
    if (frame_is(frame, "AA 55 00 04 FB")) return Command::kForward;
    if (frame_is(frame, "AA 55 00 05 FB")) return Command::kBack;
    if (frame_is(frame, "AA 55 00 06 FB")) return Command::kLeft;
    if (frame_is(frame, "AA 55 00 07 FB")) return Command::kRight;

    const std::string raw(reinterpret_cast<const char*>(frame.data()), frame.size());
    Command cmd = command_from_text(raw);
    if (cmd != Command::kUnknown) return cmd;

    const std::string ascii = bytes_to_ascii(frame);
    return command_from_text(ascii);
}

class SwappedMotorController {
public:
    SwappedMotorController()
        : physical_pwm_1_(ZF_GPIO_MOTOR_1, O_RDWR),
          physical_pwm_2_(ZF_GPIO_MOTOR_2, O_RDWR),
          physical_dir_1_(ZF_PWM_MOTOR_1),
          physical_dir_2_(ZF_PWM_MOTOR_2),
          encoder_1_(ZF_ENCODER_QUAD_1),
          encoder_2_(ZF_ENCODER_QUAD_2)
    {
        physical_dir_1_.get_dev_info(&dir1_info_);
        physical_dir_2_.get_dev_info(&dir2_info_);
    }

    void stop_all()
    {
        physical_pwm_1_.set_level(0);
        physical_pwm_2_.set_level(0);
        physical_dir_1_.set_duty(0);
        physical_dir_2_.set_duty(0);
    }

    void clear_encoders()
    {
        encoder_1_.clear_count();
        encoder_2_.clear_count();
    }

    void set_dirs(int motor1_dir, int motor2_dir)
    {
        physical_dir_1_.set_duty(static_cast<uint16>(motor1_dir ? dir1_info_.duty_max : 0));
        physical_dir_2_.set_duty(static_cast<uint16>(motor2_dir ? dir2_info_.duty_max : 0));
    }

    void pulse(bool motor1_enable, bool motor2_enable, int pulse_on_ms)
    {
        if (motor1_enable) physical_pwm_1_.set_level(1);
        if (motor2_enable) physical_pwm_2_.set_level(1);
        system_delay_ms(std::max(1, pulse_on_ms));
        physical_pwm_1_.set_level(0);
        physical_pwm_2_.set_level(0);
    }

    int read_encoder_1()
    {
        const int value = static_cast<int>(encoder_1_.get_count());
        encoder_1_.clear_count();
        return value;
    }

    int read_encoder_2()
    {
        const int value = static_cast<int>(encoder_2_.get_count());
        encoder_2_.clear_count();
        return value;
    }

private:
    zf_driver_gpio physical_pwm_1_;
    zf_driver_gpio physical_pwm_2_;
    zf_driver_pwm physical_dir_1_;
    zf_driver_pwm physical_dir_2_;
    zf_driver_encoder encoder_1_;
    zf_driver_encoder encoder_2_;
    pwm_info dir1_info_ {};
    pwm_info dir2_info_ {};
};

SwappedMotorController* g_motor = nullptr;

class SteeringServo {
public:
    SteeringServo()
        : servo_pwm_(ZF_PWM_SERVO_1)
    {
        servo_pwm_.get_dev_info(&servo_info_);
    }

    void set_angle(float angle_deg, int hold_ms)
    {
        const uint16 duty = duty_from_angle(angle_deg);
        std::printf("servo angle=%.1f duty=%u hold=%dms\n", angle_deg, duty, hold_ms);
        servo_pwm_.set_duty(duty);
        system_delay_ms(std::max(50, hold_ms));
    }

    void center(const Config& config)
    {
        set_angle(config.servo_center_deg, config.servo_settle_ms);
    }

    void release()
    {
        servo_pwm_.set_duty(0);
    }

private:
    uint16 duty_from_angle(float angle_deg) const
    {
        angle_deg = std::clamp(angle_deg, 45.0f, 135.0f);
        if (servo_info_.freq == 0 || servo_info_.duty_max == 0) {
            return 0;
        }

        const float pulse_ms = 0.5f + angle_deg / 90.0f;
        const float period_ms = 1000.0f / static_cast<float>(servo_info_.freq);
        const float duty = static_cast<float>(servo_info_.duty_max) * pulse_ms / period_ms;
        return static_cast<uint16>(std::clamp(duty, 0.0f, static_cast<float>(servo_info_.duty_max)));
    }

    zf_driver_pwm servo_pwm_;
    pwm_info servo_info_ {};
};

SteeringServo* g_servo = nullptr;

void stop_motor_from_signal()
{
    if (g_motor != nullptr) {
        g_motor->stop_all();
    }
    if (g_servo != nullptr) {
        g_servo->release();
    }
}

int inverted(int value)
{
    return value ? 0 : 1;
}

bool wait_with_stop_poll(SerialPort& serial, FrameReader& reader, const Config& config, int wait_ms)
{
    const long deadline = now_ms() + wait_ms;
    while (!g_stop_requested && now_ms() < deadline) {
        std::vector<unsigned char> frame;
        if (reader.poll(serial, 25, &frame)) {
            if (match_command(config, frame) == Command::kStop) {
                std::puts("stop command received during motion");
                return false;
            }
        }
    }
    return !g_stop_requested;
}

MotionOutcome execute_motion_core(Command cmd, SwappedMotorController& motor, SteeringServo& servo,
                                  const Config& config,
                                  const std::function<bool(int)>& wait_after_pulse)
{
    bool enable1 = true;
    bool enable2 = true;
    int motor1_dir = config.motor1_forward_dir;
    int motor2_dir = config.motor2_forward_dir;
    int target_counts = config.forward_counts;
    bool steer_motion = false;

    if (cmd == Command::kBack) {
        motor1_dir = inverted(config.motor1_forward_dir);
        motor2_dir = inverted(config.motor2_forward_dir);
    } else if (cmd == Command::kLeft) {
        servo.set_angle(config.servo_left_deg, config.servo_settle_ms);
        target_counts = config.turn_counts;
        steer_motion = true;
    } else if (cmd == Command::kRight) {
        servo.set_angle(config.servo_right_deg, config.servo_settle_ms);
        target_counts = config.turn_counts;
        steer_motion = true;
    } else if (cmd != Command::kForward) {
        return MotionOutcome {};
    }

    target_counts = std::max(1, target_counts);
    std::printf("motion begin: %s target_counts=%d dirs=(%d,%d)\n",
                command_name(cmd).c_str(), target_counts, motor1_dir, motor2_dir);
    motor.stop_all();
    system_delay_ms(100);
    motor.clear_encoders();
    motor.set_dirs(motor1_dir, motor2_dir);

    MotionOutcome outcome;
    const long deadline = now_ms() + std::max(200, config.action_timeout_ms);
    int sample = 0;

    while (!g_stop_requested && now_ms() < deadline) {
        motor.pulse(enable1, enable2, config.pulse_on_ms);
        if (!wait_after_pulse(config.pulse_off_ms)) {
            outcome.stopped = true;
            break;
        }

        const int enc1_delta = motor.read_encoder_1();
        const int enc2_delta = motor.read_encoder_2();
        outcome.enc1_total += enc1_delta;
        outcome.enc2_total += enc2_delta;
        std::printf("motion,%d,%d,%d,%d,%d\n", sample++, enc1_delta, enc2_delta, outcome.enc1_total, outcome.enc2_total);
        std::fflush(stdout);

        if (std::abs(enc1_delta) > config.max_encoder_delta || std::abs(enc2_delta) > config.max_encoder_delta) {
            std::puts("large encoder jump, stopping motion");
            break;
        }
        if (std::max(std::abs(outcome.enc1_total), std::abs(outcome.enc2_total)) >= target_counts) {
            std::puts("motion target reached");
            outcome.target_reached = true;
            break;
        }
    }

    motor.stop_all();
    motor.clear_encoders();
    if (steer_motion) {
        servo.center(config);
    }
    std::printf("motion end: %s enc=(%d,%d)\n", command_name(cmd).c_str(), outcome.enc1_total, outcome.enc2_total);
    return outcome;
}

MotionOutcome execute_motion(Command cmd, SwappedMotorController& motor, SteeringServo& servo,
                             SerialPort& serial, FrameReader& reader, const Config& config)
{
    return execute_motion_core(
        cmd, motor, servo, config,
        [&](int wait_ms) {
            return wait_with_stop_poll(serial, reader, config, wait_ms);
        });
}

std::string lower_copy(std::string text)
{
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return text;
}

std::vector<std::string> split_words(const std::string& line)
{
    std::istringstream iss(line);
    std::vector<std::string> words;
    std::string word;
    while (iss >> word) {
        words.push_back(word);
    }
    return words;
}

class UdpAgentSocket {
public:
    ~UdpAgentSocket()
    {
        close_socket();
    }

    bool open_bind(int port)
    {
        close_socket();
        fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (fd_ < 0) {
            std::printf("udp socket failed: %s\n", std::strerror(errno));
            return false;
        }

        int yes = 1;
        setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

        sockaddr_in addr {};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port = htons(static_cast<uint16_t>(port));
        if (bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
            std::printf("udp bind 0.0.0.0:%d failed: %s\n", port, std::strerror(errno));
            close_socket();
            return false;
        }

        const int flags = fcntl(fd_, F_GETFL, 0);
        fcntl(fd_, F_SETFL, flags | O_NONBLOCK);
        return true;
    }

    int fd() const
    {
        return fd_;
    }

    void close_socket()
    {
        if (fd_ >= 0) {
            close(fd_);
            fd_ = -1;
        }
    }

    bool recv_line(std::string* line, sockaddr_in* peer)
    {
        if (line == nullptr || peer == nullptr || fd_ < 0) return false;
        char buffer[512] {};
        socklen_t len = sizeof(*peer);
        const ssize_t n = recvfrom(fd_, buffer, sizeof(buffer) - 1, 0,
                                   reinterpret_cast<sockaddr*>(peer), &len);
        if (n <= 0) {
            return false;
        }
        buffer[n] = '\0';
        *line = trim(buffer);
        return true;
    }

    void send_to(const sockaddr_in& peer, const std::string& line)
    {
        if (fd_ < 0) return;
        std::string payload = line;
        if (payload.empty() || payload.back() != '\n') payload.push_back('\n');
        sendto(fd_, payload.data(), payload.size(), 0,
               reinterpret_cast<const sockaddr*>(&peer), sizeof(peer));
    }

private:
    int fd_ = -1;
};

std::string peer_to_string(const sockaddr_in& peer)
{
    char ip[INET_ADDRSTRLEN] {};
    inet_ntop(AF_INET, &peer.sin_addr, ip, sizeof(ip));
    std::ostringstream oss;
    oss << ip << ":" << ntohs(peer.sin_port);
    return oss.str();
}

Command command_from_motion_word(const std::string& word)
{
    const std::string cmd = lower_copy(word);
    if (cmd == "forward" || cmd == "fwd" || cmd == "front") return Command::kForward;
    if (cmd == "back" || cmd == "backward" || cmd == "reverse") return Command::kBack;
    if (cmd == "left") return Command::kLeft;
    if (cmd == "right") return Command::kRight;
    return Command::kUnknown;
}

bool udp_wait_with_stop_poll(UdpAgentSocket& udp, SwappedMotorController& motor,
                             int wait_ms, sockaddr_in* last_peer)
{
    const long deadline = now_ms() + wait_ms;
    while (!g_stop_requested && now_ms() < deadline) {
        const int slice = static_cast<int>(std::min<long>(25, std::max<long>(1, deadline - now_ms())));
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(udp.fd(), &rfds);
        timeval tv {};
        tv.tv_sec = slice / 1000;
        tv.tv_usec = (slice % 1000) * 1000;
        const int rc = select(udp.fd() + 1, &rfds, nullptr, nullptr, &tv);
        if (rc > 0 && FD_ISSET(udp.fd(), &rfds)) {
            sockaddr_in peer {};
            std::string line;
            if (udp.recv_line(&line, &peer)) {
                if (last_peer != nullptr) {
                    *last_peer = peer;
                }
                const std::string lowered = lower_copy(line);
                std::printf("udp during motion from %s: %s\n", peer_to_string(peer).c_str(), line.c_str());
                if (lowered == "stop" || lowered == "estop") {
                    motor.stop_all();
                    udp.send_to(peer, "ACK stop during_motion");
                    return false;
                }
                udp.send_to(peer, "BUSY motion_running");
            }
        }
    }
    return !g_stop_requested;
}

void send_agent_status(UdpAgentSocket& udp, const sockaddr_in& peer, const char* state)
{
    std::ostringstream oss;
    oss << "STATE " << state << " ms=" << now_ms();
    udp.send_to(peer, oss.str());
}

bool handle_agent_line(const std::string& line, UdpAgentSocket& udp, const sockaddr_in& peer,
                       SwappedMotorController& motor, SteeringServo& servo, Config& config)
{
    const std::vector<std::string> words = split_words(line);
    if (words.empty()) return true;
    const std::string op = lower_copy(words[0]);

    std::printf("udp command from %s: %s\n", peer_to_string(peer).c_str(), line.c_str());
    std::fflush(stdout);

    if (op == "ping" || op == "hello") {
        udp.send_to(peer, "PONG voice_motion_agent");
        return true;
    }
    if (op == "status") {
        send_agent_status(udp, peer, "idle");
        return true;
    }
    if (op == "stop" || op == "estop") {
        motor.stop_all();
        servo.center(config);
        udp.send_to(peer, "ACK stop");
        return true;
    }
    if (op == "set" && words.size() >= 3) {
        const std::string key = lower_copy(words[1]);
        const int value = parse_int(words[2], 0);
        if (key == "pulse_on_ms" || key == "pulse_on") config.pulse_on_ms = std::max(1, value);
        else if (key == "pulse_off_ms" || key == "pulse_off") config.pulse_off_ms = std::max(1, value);
        else if (key == "forward_counts") config.forward_counts = std::max(1, value);
        else if (key == "turn_counts") config.turn_counts = std::max(1, value);
        else if (key == "action_timeout_ms" || key == "timeout") config.action_timeout_ms = std::max(200, value);
        else {
            udp.send_to(peer, "ERR unknown_set_key");
            return true;
        }
        std::ostringstream oss;
        oss << "ACK set " << key << "=" << value;
        udp.send_to(peer, oss.str());
        return true;
    }
    if (op == "servo" && words.size() >= 2) {
        const std::string target = lower_copy(words[1]);
        if (target == "center") servo.center(config);
        else if (target == "left") servo.set_angle(config.servo_left_deg, config.servo_settle_ms);
        else if (target == "right") servo.set_angle(config.servo_right_deg, config.servo_settle_ms);
        else servo.set_angle(parse_float(words[1], config.servo_center_deg), config.servo_settle_ms);
        udp.send_to(peer, "ACK servo");
        return true;
    }

    Command motion = Command::kUnknown;
    int counts = 0;
    if ((op == "move" || op == "motion") && words.size() >= 2) {
        motion = command_from_motion_word(words[1]);
        counts = words.size() >= 3 ? parse_int(words[2], 0) : 0;
    } else if (op == "turn" && words.size() >= 2) {
        motion = command_from_motion_word(words[1]);
        counts = words.size() >= 3 ? parse_int(words[2], 0) : 0;
    } else {
        motion = command_from_motion_word(words[0]);
        counts = words.size() >= 2 ? parse_int(words[1], 0) : 0;
    }

    if (motion == Command::kForward || motion == Command::kBack || motion == Command::kLeft || motion == Command::kRight) {
        Config action_config = config;
        if (counts > 0) {
            if (motion == Command::kLeft || motion == Command::kRight) {
                action_config.turn_counts = counts;
            } else {
                action_config.forward_counts = counts;
            }
        }
        udp.send_to(peer, "ACK motion_begin " + command_name(motion));
        sockaddr_in last_peer = peer;
        MotionOutcome outcome = execute_motion_core(
            motion, motor, servo, action_config,
            [&](int wait_ms) {
                return udp_wait_with_stop_poll(udp, motor, wait_ms, &last_peer);
            });
        std::ostringstream oss;
        oss << "DONE " << command_name(motion)
            << " enc1=" << outcome.enc1_total
            << " enc2=" << outcome.enc2_total
            << " target=" << (outcome.target_reached ? 1 : 0)
            << " stopped=" << (outcome.stopped ? 1 : 0);
        udp.send_to(last_peer, oss.str());
        return true;
    }

    udp.send_to(peer, "ERR unknown_command");
    return true;
}

std::string get_arg(int argc, char** argv, const std::string& name, const std::string& fallback)
{
    for (int i = 2; i + 1 < argc; ++i) {
        if (argv[i] == name) {
            return argv[i + 1];
        }
    }
    return fallback;
}

int get_int_arg(int argc, char** argv, const std::string& name, int fallback)
{
    return parse_int(get_arg(argc, argv, name, std::to_string(fallback)), fallback);
}

int run_probe(int argc, char** argv)
{
    Config config;
    const std::string port = get_arg(argc, argv, "--port", config.voice_port);
    const int baud = get_int_arg(argc, argv, "--baud", config.baud);
    const int seconds = get_int_arg(argc, argv, "--seconds", 60);

    SerialPort serial;
    if (!serial.open_port(port, baud)) return 1;
    FrameReader reader;
    const long deadline = now_ms() + std::max(1, seconds) * 1000L;

    std::printf("probe listening on %s baud=%d for %d seconds\n", port.c_str(), baud, seconds);
    while (!g_stop_requested && now_ms() < deadline) {
        std::vector<unsigned char> frame;
        if (reader.poll(serial, 50, &frame)) {
            std::printf("frame len=%zu hex=[%s] ascii=[%s]\n",
                        frame.size(), bytes_to_hex(frame).c_str(), bytes_to_ascii(frame).c_str());
            std::fflush(stdout);
        }
    }
    return 0;
}

int run_learn(int argc, char** argv)
{
    Config config;
    const std::string config_path = get_arg(argc, argv, "--config", kDefaultConfigPath);
    load_config(config_path, &config);
    config.voice_port = get_arg(argc, argv, "--port", config.voice_port);
    config.baud = get_int_arg(argc, argv, "--baud", config.baud);

    SerialPort serial;
    if (!serial.open_port(config.voice_port, config.baud)) return 1;
    FrameReader reader;

    std::puts("learn mode: speak each command after the prompt.");
    for (Command cmd : {Command::kStart, Command::kStop, Command::kForward, Command::kBack, Command::kLeft, Command::kRight}) {
        std::printf("Please speak command [%s] within 20 seconds...\n", command_name(cmd).c_str());
        std::fflush(stdout);
        std::vector<unsigned char> frame;
        if (!reader.wait_frame(serial, 20000, &frame)) {
            std::printf("timeout learning %s\n", command_name(cmd).c_str());
            return 2;
        }
        config.command_frames[cmd] = frame;
        std::printf("learned %s: hex=[%s] ascii=[%s]\n",
                    command_name(cmd).c_str(), bytes_to_hex(frame).c_str(), bytes_to_ascii(frame).c_str());
    }

    return save_config(config_path, config) ? 0 : 1;
}

int run_control(int argc, char** argv)
{
    Config config;
    const std::string config_path = get_arg(argc, argv, "--config", kDefaultConfigPath);
    load_config(config_path, &config);
    config.voice_port = get_arg(argc, argv, "--port", config.voice_port);
    config.baud = get_int_arg(argc, argv, "--baud", config.baud);

    SerialPort serial;
    if (!serial.open_port(config.voice_port, config.baud)) return 1;

    SwappedMotorController motor;
    SteeringServo servo;
    g_motor = &motor;
    g_servo = &servo;
    motor.stop_all();
    motor.clear_encoders();
    servo.center(config);

    FrameReader reader;
    bool armed = false;
    std::printf("run mode: port=%s baud=%d state=DISARMED\n", config.voice_port.c_str(), config.baud);
    std::fflush(stdout);

    while (!g_stop_requested) {
        std::vector<unsigned char> frame;
        if (!reader.poll(serial, 100, &frame)) {
            continue;
        }

        const Command cmd = match_command(config, frame);
        std::printf("voice frame hex=[%s] ascii=[%s] command=%s state=%s\n",
                    bytes_to_hex(frame).c_str(), bytes_to_ascii(frame).c_str(),
                    command_name(cmd).c_str(), armed ? "ARMED" : "DISARMED");
        std::fflush(stdout);

        if (cmd == Command::kStart) {
            armed = true;
            std::puts("state=ARMED");
        } else if (cmd == Command::kStop) {
            motor.stop_all();
            armed = false;
            std::puts("state=DISARMED");
        } else if (armed && (cmd == Command::kForward || cmd == Command::kBack ||
                            cmd == Command::kLeft || cmd == Command::kRight)) {
            execute_motion(cmd, motor, servo, serial, reader, config);
            armed = false;
            std::puts("motion complete, state=DISARMED");
        } else {
            std::puts("ignored");
        }
    }

    motor.stop_all();
    servo.center(config);
    servo.release();
    g_motor = nullptr;
    g_servo = nullptr;
    return 0;
}

int run_agent(int argc, char** argv)
{
    Config config;
    const std::string config_path = get_arg(argc, argv, "--config", kDefaultConfigPath);
    load_config(config_path, &config);
    config.agent_bind_port = get_int_arg(argc, argv, "--port", config.agent_bind_port);

    UdpAgentSocket udp;
    if (!udp.open_bind(config.agent_bind_port)) return 1;

    SwappedMotorController motor;
    SteeringServo servo;
    g_motor = &motor;
    g_servo = &servo;
    motor.stop_all();
    motor.clear_encoders();
    servo.center(config);

    sockaddr_in last_peer {};
    bool has_peer = false;
    long last_status_ms = now_ms();

    std::printf("agent mode: udp listen 0.0.0.0:%d\n", config.agent_bind_port);
    std::printf("protocol examples: PING | STATUS | STOP | MOVE forward 30 | MOVE back 30 | TURN left 16 | SERVO center\n");
    std::fflush(stdout);

    while (!g_stop_requested) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(udp.fd(), &rfds);
        timeval tv {};
        tv.tv_sec = 0;
        tv.tv_usec = 100000;
        const int rc = select(udp.fd() + 1, &rfds, nullptr, nullptr, &tv);
        if (rc > 0 && FD_ISSET(udp.fd(), &rfds)) {
            sockaddr_in peer {};
            std::string line;
            if (udp.recv_line(&line, &peer)) {
                last_peer = peer;
                has_peer = true;
                handle_agent_line(line, udp, peer, motor, servo, config);
            }
        }

        const long t = now_ms();
        if (has_peer && config.agent_status_period_ms > 0 &&
            (t - last_status_ms) >= config.agent_status_period_ms) {
            send_agent_status(udp, last_peer, "idle");
            last_status_ms = t;
        }
    }

    motor.stop_all();
    servo.center(config);
    servo.release();
    g_motor = nullptr;
    g_servo = nullptr;
    return 0;
}

void print_usage(const char* argv0)
{
    std::printf("usage:\n");
    std::printf("  %s probe [--port /dev/ttyS1] [--baud 115200] [--seconds 60]\n", argv0);
    std::printf("  %s learn [--config /home/root/voice_motion_config.ini] [--port /dev/ttyS1] [--baud 115200]\n", argv0);
    std::printf("  %s run [--config /home/root/voice_motion_config.ini] [--port /dev/ttyS1] [--baud 115200]\n", argv0);
    std::printf("  %s agent [--config /home/root/voice_motion_config.ini] [--port 15000]\n", argv0);
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
    if (mode == "probe") {
        return run_probe(argc, argv);
    }
    if (mode == "learn") {
        return run_learn(argc, argv);
    }
    if (mode == "run") {
        const int rc = run_control(argc, argv);
        stop_motor_from_signal();
        return rc;
    }
    if (mode == "agent") {
        const int rc = run_agent(argc, argv);
        stop_motor_from_signal();
        return rc;
    }

    print_usage(argv[0]);
    return 2;
}
