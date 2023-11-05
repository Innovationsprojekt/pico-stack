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
