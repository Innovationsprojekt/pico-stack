/**
 * @file servo.cpp
 * @author Noa Sendlhofer
 */

#include <stdexcept>
#include "servo.h"

void Servo::setAngle(double angle, uint8_t speed)
{
    if (angle > MAX_ANGLE || angle < 0)
        throw(std::invalid_argument("Angle out of bounds"));

    uint32_t pos = _angleToPosition(angle);

    if (pos > getCurrentPWM())
    {
        for (uint32_t i = getCurrentPWM(); i < pos; i = i+speed)
        {
            writePWM(i);
            sleep_ms(SERVO_SPEED);
        }
    } else
    {
        for (uint32_t i = getCurrentPWM(); i > pos; i = i-speed)
        {
            writePWM(i);
            sleep_ms(SERVO_SPEED);
        }
    }
}

bool Servo::spinSetAngle(double angle, uint8_t speed)
{
    if (angle > MAX_ANGLE || angle < 0)
        throw(std::invalid_argument("Angle out of bounds"));

    int32_t set_pos = _angleToPosition(angle);

    if (abs(set_pos - int32_t(getCurrentPWM())) < 5)
        return true;

    if (set_pos > getCurrentPWM())
        writePWM(getCurrentPWM() + speed);
    else
        writePWM(getCurrentPWM() - speed);

    return false;
}

void Servo::_setAngle(double angle)
{
    printf("_ Set Angle");
    if (angle > MAX_ANGLE || angle < 0)
        throw(std::invalid_argument("Angle out of bounds"));

    writePWM(_angleToPosition(angle));
}

double Servo::getCurrentAngle()
{
    return _positionToAngle(getCurrentPWM());
}

uint16_t Servo::_angleToPosition(double angle)
{
    double part = angle / MAX_ANGLE;
    double range = SERVO_MAX - SERVO_MIN;

    double div = range*part + SERVO_MIN;

    return uint16_t(div);
}

double Servo::_positionToAngle(uint16_t position)
{
    double range = SERVO_MAX - SERVO_MIN;
    double part = double(position - SERVO_MIN) / range;

    return MAX_ANGLE*part;
}