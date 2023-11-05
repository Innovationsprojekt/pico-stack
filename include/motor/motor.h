/**
 * @file motor.h
 * @author Noa Sendlhofer
 */

#ifndef PICO_MOTORS_MOTOR_H_
#define PICO_MOTORS_MOTOR_H_

#include "motor_driver.h"

#define MAX_SPEED 10000
#define MIN_SPEED 0

#define BASE_WRAP 10000

class Motor : MotorDriver
{
public:
    Motor(uint8_t step_pin, uint8_t dir_pin)
        : MotorDriver(step_pin, 0, BASE_WRAP)
        , dir_pin(dir_pin)
    {
        gpio_init(dir_pin);
        gpio_set_dir(dir_pin, GPIO_OUT);
        setDirection(STOP);
    }

    void setSpeed(int32_t speed);
    uint32_t getCurrentSpeed();

    void setDirection(MotorDirection direction);
    void changeDirection();
    MotorDirection getCurrentDirection() const;

private:
    void _writeDirection(MotorDirection direction) const;

    const uint8_t dir_pin;

    MotorDirection current_dir = STOP;
};

#endif //PICO_MOTORS_MOTOR_H_
