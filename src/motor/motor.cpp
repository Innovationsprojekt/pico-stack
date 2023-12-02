/**
 * @file motor.cpp
 * @author Noa Sendlhofer
 */

#include "motor.h"

void Motor::setDirection(MotorDirection direction)
{
    switch (direction)
    {
        case FORWARD:
            _writeDirection(FORWARD);
            this->current_dir = FORWARD;
            break;
        case BACKWARD:
            _writeDirection(BACKWARD);
            this->current_dir = BACKWARD;
            break;
        case STOP:
            setSpeed(0);
            this->current_dir = STOP;
            break;
    }
}

void Motor::_writeDirection(MotorDirection direction) const
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

MotorDirection Motor::getCurrentDirection() const
{
    return current_dir;
}

void Motor::setSpeed(int32_t speed)
{
    if (speed > MAX_SPEED)
        speed = MAX_SPEED;

    if (speed < MIN_SPEED)
        speed = MIN_SPEED;

    writePWM(speed*BASE_WRAP/MAX_SPEED);
}

uint32_t Motor::getCurrentSpeed()
{
    return getCurrentPWM();
}
