/**
 * @file stepper_driver.cpp
 * @author Noa Sendlhofer
 */

#include "motor_driver.h"
#include <stdlib.h>

Motor::Motor(uint8_t step_pin, uint8_t dir_pin)
    : step_pin(step_pin)
    , dir_pin(dir_pin)
{
    gpio_set_function(step_pin, GPIO_FUNC_PWM);

    gpio_init(dir_pin);
    gpio_set_dir(dir_pin, GPIO_OUT);

    slice_num = pwm_gpio_to_slice_num(step_pin);
    pwm_channel = pwm_gpio_to_channel(step_pin);

    setSpeed(0);
    setEnabled(true);
}

void Motor::setDirection(MotorDirection direction)
{
    switch (direction)
    {
        case FORWARD:
            writeDirection(FORWARD);
            this->current_dir = FORWARD;
            break;
        case BACKWARD:
            writeDirection(BACKWARD);
            this->current_dir = BACKWARD;
            break;
        case STOP:
            setSpeed(0);
            break;
    }
}

void Motor::setSpeed(int32_t speed)
{
    if (speed > 5000)
        speed = 5000;

    if (speed < 0)
        speed = 0;

    current_speed = speed;
    pwm_set_wrap(slice_num, 10000);
    pwm_set_chan_level(slice_num, pwm_channel, int(float(speed)/10000.0*10000.0));
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

void Motor::changeDirection()
{
    if (current_dir == FORWARD)
        setDirection(BACKWARD);
    else
        setDirection(FORWARD);
}

uint8_t Motor::getCurrentSpeed() const
{
    return this->current_speed;
}

MotorDirection Motor::getCurrentDirection() const
{
    return current_dir;
}
