/**
 * @file stepper_driver.cpp
 * @author Noa Sendlhofer
 */

#include "motor_driver.h"

Motor::Motor(uint8_t step_pin, uint8_t dir_pin)
    : step_pin(step_pin)
    , dir_pin(dir_pin)
{
    gpio_set_function(step_pin, GPIO_FUNC_PWM);

    gpio_init(dir_pin);
    gpio_set_dir(dir_pin, GPIO_OUT);

    slice_num = pwm_gpio_to_slice_num(step_pin);
    pwm_channel = pwm_gpio_to_channel(step_pin);
}

void Motor::setDirection(MotorDirection direction)
{
    switch (direction)
    {
        case FORWARD:
            setEnabled(true);
            writeDirection(FORWARD);
        case BACKWARD:
            setEnabled(true);
            writeDirection(BACKWARD);
            break;
        case STOP:
            setEnabled(false);
            break;
    }
}

void Motor::setSpeed(uint8_t speed) const
{
    pwm_set_wrap(slice_num, speed);
    pwm_set_chan_level(slice_num, pwm_channel, speed/2);
}

void Motor::setEnabled(bool state)
{
    if (state && !enabled)
        pwm_set_enabled(slice_num, true);
    else if (!state && enabled)
        pwm_set_enabled(slice_num, false);

    enabled = state;
}

void Motor::writeDirection(MotorDirection direction) const
{
    switch (direction)
    {
        case FORWARD:
            gpio_put(dir_pin, 1);
            break;
        case BACKWARD:
            gpio_put(dir_pin, 0);
            break;
        default:
            break;
    }
}
