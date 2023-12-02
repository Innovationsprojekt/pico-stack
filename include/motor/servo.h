/**
 * @file servo.h
 * @author Noa Sendlhofer
 */

#ifndef PICO_MOTORS_SERVO_H_
#define PICO_MOTORS_SERVO_H_

#include "motor_driver.h"

#define SERVO_WRAP 12500
#define CLK_DIV 200

#define SERVO_MIN 340
#define SERVO_MAX 1700
#define MAX_ANGLE 203.0

class Servo : MotorDriver
{
public:
    Servo(uint8_t step_pin)
        : MotorDriver(step_pin, CLK_DIV, SERVO_WRAP)
    {
        _setAngle(130);
    }

    void setAngle(double angle, uint8_t speed = SERVO_SPEED);
    bool spinSetAngle(double angle, uint8_t speed);
    double getCurrentAngle();

private:
    void _setAngle(double angle);

    static uint16_t _angleToPosition(double angle);
    static double _positionToAngle(uint16_t position);

    uint16_t current_position = 0;
    int32_t _current_pwm = 0;
};

#endif //PICO_MOTORS_SERVO_H_