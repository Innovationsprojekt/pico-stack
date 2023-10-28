/**
 * @file stepper_driver.h
 * @author Noa Sendlhofer
 */

#ifndef PICO_MOTORS_MOTOR_DRIVER_H_
#define PICO_MOTORS_MOTOR_DRIVER_H_

#include <cstdint>
#include <hardware/pwm.h>
#include <pico/stdlib.h>

enum MotorDirection
{
    FORWARD = 0,
    BACKWARD = 1,
    STOP = 2,
};

class Motor
{
public:
    Motor(uint8_t step_pin, uint8_t dir_pin);
    void setDirection(MotorDirection direction);
    void setSpeed(int32_t speed);
    void changeDirection();
    uint8_t getCurrentSpeed() const;
    MotorDirection getCurrentDirection() const;

private:
    void setEnabled(bool state);
    void writeDirection(MotorDirection direction) const;

    const uint8_t step_pin;
    const uint8_t dir_pin;

    uint slice_num;
    uint pwm_channel;

    bool enabled = false;
    uint32_t current_speed = 0;
    MotorDirection current_dir = STOP;
};

#endif //PICO_MOTORS_MOTOR_DRIVER_H_
