#include "zf_driver_gpio.hpp"
#include "zf_driver_pwm.hpp"
#include "zf_driver_delay.hpp"

#include <csignal>
#include <cstdio>
#include <cstdlib>

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

uint16 duty_from_percent(const pwm_info& info, int percent)
{
    if (percent < 0) percent = 0;
    if (percent > 10) percent = 10;
    return static_cast<uint16>(percent * (info.duty_max / 100));
}

void pulse_motor(const char* name, zf_driver_gpio& dir, zf_driver_pwm& pwm,
                 const pwm_info& info, int percent)
{
    const uint16 duty = duty_from_percent(info, percent);
    std::printf("%s: dir=1 duty=%u (%d%%) for 1s\n", name, duty, percent);
    dir.set_level(1);
    pwm.set_duty(duty);
    system_delay_ms(1000);
    pwm.set_duty(0);
    std::printf("%s: stopped\n", name);
    system_delay_ms(800);
}

}  // namespace

int main(int argc, char** argv)
{
    int percent = 5;
    if (argc >= 2) {
        percent = std::atoi(argv[1]);
    }
    if (percent < 1 || percent > 10) {
        std::puts("usage: safe_motor_test [1..10]");
        return 2;
    }

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    pwm1.get_dev_info(&pwm1_info);
    pwm2.get_dev_info(&pwm2_info);

    std::printf("pwm1 duty_max=%u freq=%u\n", pwm1_info.duty_max, pwm1_info.freq);
    std::printf("pwm2 duty_max=%u freq=%u\n", pwm2_info.duty_max, pwm2_info.freq);
    std::printf("safe motor test percent=%d\n", percent);

    stop_all();
    system_delay_ms(500);

    pulse_motor("motor1", dir1, pwm1, pwm1_info, percent);
    pulse_motor("motor2", dir2, pwm2, pwm2_info, percent);

    stop_all();
    std::puts("safe motor test done, all motors stopped");
    return 0;
}
