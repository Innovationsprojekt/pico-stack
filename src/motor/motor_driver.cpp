/**
 * @file stepper_driver.cpp
 * @author Noa Sendlhofer
 */

#include <stdexcept>
#include "motor_driver.h"

MotorDriver::MotorDriver(uint8_t step_pin, uint8_t clk_div, uint16_t wrap)
    : step_pin(step_pin)
    , clk_div(clk_div)
    , wrap(wrap)
{
    gpio_set_function(step_pin, GPIO_FUNC_PWM);

    slice_num = pwm_gpio_to_slice_num(step_pin);
    pwm_channel = pwm_gpio_to_channel(step_pin);

    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, clk_div);
    pwm_config_set_wrap(&config, wrap);

    pwm_init(slice_num, &config, true);

    writePWM(0);
    _setEnabled(true);
}

void MotorDriver::writePWM(int32_t pwm_width)
{
    if (pwm_width < 0 || pwm_width > wrap)
        throw(std::invalid_argument("pwm_width out of bounds"));

    current_pwm = pwm_width;
    pwm_set_chan_level(slice_num, pwm_channel, pwm_width);
}

void MotorDriver::_setEnabled(bool state)
{
    if (state && !enabled)
        pwm_set_enabled(slice_num, true);
    else if (!state && enabled)
        pwm_set_enabled(slice_num, false);

    enabled = state;
}

uint32_t MotorDriver::getCurrentPWM() const
{
    return current_pwm;
}
