/**
 * @file cl_motor.h
 * @author Noa Sendlhofer
 */

#ifndef PICO_MOTORS_CL_MOTOR_H_
#define PICO_MOTORS_CL_MOTOR_H_

#include <memory>
#include "motor_driver.h"
#include "encoder.h"

#define MAX_SPEED 10000
#define MIN_SPEED 0

#define BASE_WRAP 10000
#define CLK_DIV 10

#define CTRL_MAX_SPEED 3000
#define CTRL_MIN_SPEED 700
#define CTRL_KP 25
#define CTRL_KD 10
#define CTRL_KI 0.05

class CLMotor : MotorDriver
{
public:
    CLMotor(uint8_t step_pin, uint8_t dir_pin, std::shared_ptr<BaseEncoder> encoder);

    void setPosition(int32_t position, int32_t speed);
    bool spinSetPosition(int32_t position, int32_t speed);

    int32_t getCurrentPosition();
    void spinController(double dt);
    void stop();

private:
    void _writeDirection(MotorDirection direction) const;
    void _setDirection(MotorDirection direction);
    void _setSpeed(int32_t speed);

    void _driveOpenLoop();
    void _driveClosedLoop(double dt);

    //std::unique_ptr<RotaryEncoder> encoder;
    std::shared_ptr<BaseEncoder> _encoder;

    const uint8_t _dir_pin;
    MotorDirection _current_dir = STOP;

    int32_t _last_error = 0;
    int32_t _integral = 0;

    int32_t _set_position = 0;
    int32_t _set_speed = 0;
    int32_t _max_speed = MAX_SPEED;
    int32_t _min_speed = MIN_SPEED;
};


#endif //PICO_MOTORS_CL_MOTOR_H_
