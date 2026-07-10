#include "zf_driver_gpio.hpp"
#include "zf_driver_pwm.hpp"
#include "zf_driver_delay.hpp"

#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace {

zf_driver_gpio dir1(ZF_GPIO_MOTOR_1, O_RDWR);
zf_driver_gpio dir2(ZF_GPIO_MOTOR_2, O_RDWR);
zf_driver_pwm pwm1(ZF_PWM_MOTOR_1);
zf_driver_pwm pwm2(ZF_PWM_MOTOR_2);

pwm_info pwm1_info {};
pwm_info pwm2_info {};

void stop_all()
{
    pwm1.set_duty(0);
    pwm2.set_duty(0);
}

void on_signal(int)
{
    stop_all();
    std::puts("signal received, motors stopped");
    std::exit(130);
}

uint16 duty_from_percent(const pwm_info& info, double percent)
{
    if (percent <= 0.0) {
        return 0;
    }
    if (percent > 10.0) {
        percent = 10.0;
    }

    double raw = percent * static_cast<double>(info.duty_max) / 100.0;
    if (raw < 1.0) {
        raw = 1.0;
    }
    return static_cast<uint16>(raw + 0.5);
}

void pulse_motor(const char* name, zf_driver_gpio& dir, zf_driver_pwm& pwm,
                 const pwm_info& info, double percent, int milliseconds)
{
    const uint16 duty = duty_from_percent(info, percent);
    std::printf("%s: dir=1 duty=%u (%.3f%%) for %d ms\n",
                name, duty, percent, milliseconds);
    dir.set_level(1);
    pwm.set_duty(duty);
    system_delay_ms(milliseconds);
    pwm.set_duty(0);
    std::printf("%s: stopped\n", name);
}

void print_usage(const char* program)
{
    std::printf("usage: %s <1|2|both|seq> <percent 0.05..10> [milliseconds 100..2000]\n", program);
    std::puts("examples:");
    std::printf("  %s 1 0.2 300   # motor1 only, 0.2%%, 300 ms\n", program);
    std::printf("  %s 2 0.2 300   # motor2 only, 0.2%%, 300 ms\n", program);
    std::printf("  %s seq 0.2 300 # motor1 then motor2\n", program);
}

}  // namespace

int main(int argc, char** argv)
{
    if (argc < 3 || argc > 4) {
        print_usage(argv[0]);
        return 2;
    }

    double percent = std::strtod(argv[2], nullptr);
    if (percent < 0.05 || percent > 10.0) {
        std::puts("percent must be in [0.05, 10]");
        return 2;
    }

    int milliseconds = 300;
    if (argc >= 4) {
        milliseconds = std::atoi(argv[3]);
    }
    if (milliseconds < 100 || milliseconds > 2000) {
        std::puts("milliseconds must be in [100, 2000]");
        return 2;
    }

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    pwm1.get_dev_info(&pwm1_info);
    pwm2.get_dev_info(&pwm2_info);

    std::printf("pwm1 duty_max=%u freq=%u\n", pwm1_info.duty_max, pwm1_info.freq);
    std::printf("pwm2 duty_max=%u freq=%u\n", pwm2_info.duty_max, pwm2_info.freq);
    std::printf("micro motor test target=%s percent=%.3f ms=%d\n",
                argv[1], percent, milliseconds);

    stop_all();
    system_delay_ms(300);

    if (std::strcmp(argv[1], "1") == 0) {
        pulse_motor("motor1", dir1, pwm1, pwm1_info, percent, milliseconds);
    } else if (std::strcmp(argv[1], "2") == 0) {
        pulse_motor("motor2", dir2, pwm2, pwm2_info, percent, milliseconds);
    } else if (std::strcmp(argv[1], "both") == 0) {
        const uint16 duty1 = duty_from_percent(pwm1_info, percent);
        const uint16 duty2 = duty_from_percent(pwm2_info, percent);
        std::printf("both: duty1=%u duty2=%u (%.3f%%) for %d ms\n",
                    duty1, duty2, percent, milliseconds);
        dir1.set_level(1);
        dir2.set_level(1);
        pwm1.set_duty(duty1);
        pwm2.set_duty(duty2);
        system_delay_ms(milliseconds);
        stop_all();
        std::puts("both: stopped");
    } else if (std::strcmp(argv[1], "seq") == 0) {
        pulse_motor("motor1", dir1, pwm1, pwm1_info, percent, milliseconds);
        system_delay_ms(2000);
        pulse_motor("motor2", dir2, pwm2, pwm2_info, percent, milliseconds);
    } else {
        print_usage(argv[0]);
        stop_all();
        return 2;
    }

    stop_all();
    std::puts("micro motor test done, all motors stopped");
    return 0;
}
