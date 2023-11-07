/**
 * @file servo.cpp
 * @author Noa Sendlhofer
 */

#include <stdexcept>
#include "servo.h"

void Servo::setAngle(double angle)
{
    if (angle > MAX_ANGLE || angle < 0)
        throw(std::invalid_argument("Angle out of bounds"));

    uint32_t pos = _angleToPosition(angle);

    printf("pos %i, current %li", pos, getCurrentPWM());

    if (pos > getCurrentPWM())
    {
        for (uint32_t i = getCurrentPWM(); i < pos; i = i+5)
        {
            printf("pos %i, current %li", i, getCurrentPWM());
            writePWM(i);
            sleep_ms(5);
        }
    } else
    {
        for (uint32_t i = getCurrentPWM(); i > pos; i = i-5)
        {
            printf("pos %i, current %li", i, getCurrentPWM());
            writePWM(i);
            sleep_ms(5);
        }
    }
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
