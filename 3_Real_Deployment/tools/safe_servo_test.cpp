#include "zf_driver_delay.hpp"
#include "zf_driver_pwm.hpp"

#include <algorithm>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace {

constexpr const char* kServoPath = ZF_PWM_SERVO_1;
constexpr float kDefaultCenterDeg = 90.0f;
constexpr float kDefaultLeftDeg = 80.0f;
constexpr float kDefaultRightDeg = 100.0f;
constexpr float kMinSafeDeg = 45.0f;
constexpr float kMaxSafeDeg = 135.0f;

zf_driver_pwm servo_pwm(kServoPath);
pwm_info servo_info {};

void release_servo()
{
    servo_pwm.set_duty(0);
}

void on_signal(int)
{
    release_servo();
    std::puts("signal received, servo PWM released");
    std::exit(130);
}

uint16 duty_from_angle(float angle_deg)
{
    angle_deg = std::clamp(angle_deg, kMinSafeDeg, kMaxSafeDeg);

    // Standard hobby servo: 0.5 ms to 2.5 ms pulse for 0 to 180 degrees.
    const float pulse_ms = 0.5f + angle_deg / 90.0f;
    const float period_ms = 1000.0f / static_cast<float>(servo_info.freq);
    const float duty = static_cast<float>(servo_info.duty_max) * pulse_ms / period_ms;
    return static_cast<uint16>(std::clamp(duty, 0.0f, static_cast<float>(servo_info.duty_max)));
}

void set_angle(float angle_deg, int hold_ms)
{
    const uint16 duty = duty_from_angle(angle_deg);
    std::printf("servo: angle=%.1f deg duty=%u hold=%d ms\n", angle_deg, duty, hold_ms);
    servo_pwm.set_duty(duty);
    system_delay_ms(hold_ms);
}

void print_usage(const char* program)
{
    std::printf("usage: %s center [hold_ms]\n", program);
    std::printf("       %s angle <deg %.0f..%.0f> [hold_ms]\n", program, kMinSafeDeg, kMaxSafeDeg);
    std::printf("       %s sweep [left_deg right_deg cycles]\n", program);
    std::puts("examples:");
    std::printf("  %s center 1000\n", program);
    std::printf("  %s angle 90 1000\n", program);
    std::printf("  %s sweep 80 100 2\n", program);
}

}  // namespace

int main(int argc, char** argv)
{
    if (argc < 2) {
        print_usage(argv[0]);
        return 2;
    }

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    servo_pwm.get_dev_info(&servo_info);
    std::printf("servo path=%s\n", kServoPath);
    std::printf("servo pwm freq=%u Hz duty_max=%u\n", servo_info.freq, servo_info.duty_max);

    if (servo_info.freq == 0 || servo_info.duty_max == 0) {
        std::puts("invalid servo pwm info");
        return 1;
    }

    if (std::strcmp(argv[1], "center") == 0) {
        const int hold_ms = (argc >= 3) ? std::atoi(argv[2]) : 1000;
        set_angle(kDefaultCenterDeg, std::clamp(hold_ms, 100, 5000));
    } else if (std::strcmp(argv[1], "angle") == 0) {
        if (argc < 3) {
            print_usage(argv[0]);
            return 2;
        }
        const float angle = std::strtof(argv[2], nullptr);
        const int hold_ms = (argc >= 4) ? std::atoi(argv[3]) : 1000;
        set_angle(angle, std::clamp(hold_ms, 100, 5000));
    } else if (std::strcmp(argv[1], "sweep") == 0) {
        float left = kDefaultLeftDeg;
        float right = kDefaultRightDeg;
        int cycles = 2;
        if (argc >= 4) {
            left = std::strtof(argv[2], nullptr);
            right = std::strtof(argv[3], nullptr);
        }
        if (argc >= 5) {
            cycles = std::atoi(argv[4]);
        }

        left = std::clamp(left, kMinSafeDeg, kMaxSafeDeg);
        right = std::clamp(right, kMinSafeDeg, kMaxSafeDeg);
        cycles = std::clamp(cycles, 1, 5);

        set_angle(kDefaultCenterDeg, 600);
        for (int i = 0; i < cycles; ++i) {
            set_angle(left, 600);
            set_angle(kDefaultCenterDeg, 400);
            set_angle(right, 600);
            set_angle(kDefaultCenterDeg, 400);
        }
    } else {
        print_usage(argv[0]);
        return 2;
    }

    release_servo();
    std::puts("safe servo test done, servo PWM released");
    return 0;
}
