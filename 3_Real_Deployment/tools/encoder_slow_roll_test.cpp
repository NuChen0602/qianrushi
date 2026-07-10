#include "zf_driver_encoder.hpp"
#include "zf_driver_gpio.hpp"
#include "zf_driver_pwm.hpp"
#include "zf_driver_delay.hpp"

#include <algorithm>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace {

zf_driver_gpio g_dir1(ZF_GPIO_MOTOR_1, O_RDWR);
zf_driver_gpio g_dir2(ZF_GPIO_MOTOR_2, O_RDWR);
zf_driver_pwm g_pwm1(ZF_PWM_MOTOR_1);
zf_driver_pwm g_pwm2(ZF_PWM_MOTOR_2);
zf_driver_encoder g_enc1(ZF_ENCODER_QUAD_1);
zf_driver_encoder g_enc2(ZF_ENCODER_QUAD_2);

volatile sig_atomic_t g_stop_requested = 0;

void stop_all()
{
    g_pwm1.set_duty(0);
    g_pwm2.set_duty(0);
    g_dir1.set_level(0);
    g_dir2.set_level(0);
}

void on_signal(int)
{
    g_stop_requested = 1;
    stop_all();
}

int abs_i(int value)
{
    return value < 0 ? -value : value;
}

void print_usage(const char* argv0)
{
    std::printf("usage: %s [motor1|motor2|both|swap1|swap2|swapboth] [target_counts] [raw_duty] [dir] [max_ms] [pulse_on_ms] [pulse_off_ms]\n", argv0);
    std::puts("example: encoder_slow_roll_test motor1 2 10 1 1500 8 180");
    std::puts("example for swapped DIR/PWM wiring: encoder_slow_roll_test swapboth 1 1 1 1200 1 800");
    std::puts("raw_duty is clamped to 1..300. duty_max is usually 10000, so 60 is about 0.6%.");
    std::puts("pulse mode is enabled by default. Use pulse_on_ms=0 for continuous output.");
}

int parse_int(const char* text, int fallback)
{
    if (text == nullptr || *text == '\0') {
        return fallback;
    }
    char* end = nullptr;
    const long value = std::strtol(text, &end, 10);
    if (end == text || *end != '\0') {
        return fallback;
    }
    return static_cast<int>(value);
}

}  // namespace

int main(int argc, char** argv)
{
    const char* mode = argc >= 2 ? argv[1] : "motor1";
    if (std::strcmp(mode, "-h") == 0 || std::strcmp(mode, "--help") == 0) {
        print_usage(argv[0]);
        return 0;
    }

    const bool swapped_wiring = std::strcmp(mode, "swap1") == 0 ||
                                std::strcmp(mode, "swap2") == 0 ||
                                std::strcmp(mode, "swapboth") == 0;
    const bool use_motor1 = std::strcmp(mode, "motor1") == 0 || std::strcmp(mode, "both") == 0 ||
                            std::strcmp(mode, "swap1") == 0 || std::strcmp(mode, "swapboth") == 0;
    const bool use_motor2 = std::strcmp(mode, "motor2") == 0 || std::strcmp(mode, "both") == 0 ||
                            std::strcmp(mode, "swap2") == 0 || std::strcmp(mode, "swapboth") == 0;
    if (!use_motor1 && !use_motor2) {
        print_usage(argv[0]);
        return 2;
    }

    const int target_counts = std::max(1, std::min(parse_int(argc >= 3 ? argv[2] : nullptr, 8), 100));
    const int raw_duty = std::max(1, std::min(parse_int(argc >= 4 ? argv[3] : nullptr, 60), 300));
    const int dir_level = parse_int(argc >= 5 ? argv[4] : nullptr, 1) ? 1 : 0;
    const int max_ms = std::max(200, std::min(parse_int(argc >= 6 ? argv[5] : nullptr, 1500), 5000));
    const int pulse_on_ms = std::max(0, std::min(parse_int(argc >= 7 ? argv[6] : nullptr, 10), 100));
    const int pulse_off_ms = std::max(0, std::min(parse_int(argc >= 8 ? argv[7] : nullptr, 180), 1000));
    const int sample_ms = 20;
    const bool pulse_mode = pulse_on_ms > 0;

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    pwm_info pwm1_info {};
    pwm_info pwm2_info {};
    g_pwm1.get_dev_info(&pwm1_info);
    g_pwm2.get_dev_info(&pwm2_info);

    stop_all();
    system_delay_ms(200);
    g_enc1.clear_count();
    g_enc2.clear_count();

    std::printf("encoder_slow_roll_test mode=%s wiring=%s target_counts=%d raw_duty=%d dir=%d max_ms=%d pulse_on_ms=%d pulse_off_ms=%d\n",
                mode, swapped_wiring ? "swapped" : "normal", target_counts, raw_duty,
                dir_level, max_ms, pulse_on_ms, pulse_off_ms);
    std::printf("pwm1 duty_max=%u freq=%u, pwm2 duty_max=%u freq=%u\n",
                pwm1_info.duty_max, pwm1_info.freq, pwm2_info.duty_max, pwm2_info.freq);
    std::puts("sample,enc1_delta,enc2_delta,enc1_total,enc2_total");
    std::fflush(stdout);

    if (swapped_wiring) {
        if (use_motor1) {
            g_pwm1.set_duty(static_cast<uint16>(dir_level ? pwm1_info.duty_max : 0));
        }
        if (use_motor2) {
            g_pwm2.set_duty(static_cast<uint16>(dir_level ? pwm2_info.duty_max : 0));
        }
        g_dir1.set_level(0);
        g_dir2.set_level(0);
    } else {
        g_dir1.set_level(static_cast<uint8>(dir_level));
        g_dir2.set_level(static_cast<uint8>(dir_level));
    }

    if (!pulse_mode && !swapped_wiring && use_motor1) {
        g_pwm1.set_duty(static_cast<uint16>(raw_duty));
    }
    if (!pulse_mode && !swapped_wiring && use_motor2) {
        g_pwm2.set_duty(static_cast<uint16>(raw_duty));
    }

    int enc1_total = 0;
    int enc2_total = 0;
    int elapsed_ms = 0;
    int sample_index = 0;

    while (!g_stop_requested && elapsed_ms < max_ms) {
        if (pulse_mode) {
            if (swapped_wiring) {
                if (use_motor1) {
                    g_dir1.set_level(1);
                }
                if (use_motor2) {
                    g_dir2.set_level(1);
                }
            } else {
                if (use_motor1) {
                    g_pwm1.set_duty(static_cast<uint16>(raw_duty));
                }
                if (use_motor2) {
                    g_pwm2.set_duty(static_cast<uint16>(raw_duty));
                }
            }
            system_delay_ms(pulse_on_ms);
            elapsed_ms += pulse_on_ms;
            if (swapped_wiring) {
                g_dir1.set_level(0);
                g_dir2.set_level(0);
            } else {
                stop_all();
                g_dir1.set_level(static_cast<uint8>(dir_level));
                g_dir2.set_level(static_cast<uint8>(dir_level));
            }
            if (pulse_off_ms > 0) {
                system_delay_ms(pulse_off_ms);
                elapsed_ms += pulse_off_ms;
            }
        } else {
            system_delay_ms(sample_ms);
            elapsed_ms += sample_ms;
        }

        const int enc1_delta = static_cast<int>(g_enc1.get_count());
        const int enc2_delta = static_cast<int>(g_enc2.get_count());
        g_enc1.clear_count();
        g_enc2.clear_count();

        enc1_total += enc1_delta;
        enc2_total += enc2_delta;

        std::printf("%d,%d,%d,%d,%d\n", sample_index, enc1_delta, enc2_delta, enc1_total, enc2_total);
        std::fflush(stdout);
        ++sample_index;

        const int moved_counts = std::max(abs_i(enc1_total), abs_i(enc2_total));
        if (moved_counts >= target_counts) {
            std::printf("target reached: moved_counts=%d\n", moved_counts);
            break;
        }

        if (abs_i(enc1_delta) > 20 || abs_i(enc2_delta) > 20) {
            std::puts("large encoder jump detected, stopping early");
            break;
        }
    }

    stop_all();
    g_enc1.clear_count();
    g_enc2.clear_count();

    std::printf("stopped. final enc1_total=%d enc2_total=%d\n", enc1_total, enc2_total);
    return g_stop_requested ? 130 : 0;
}
