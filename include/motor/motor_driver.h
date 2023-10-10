/**
 * @file stepper_driver.h
 * @author Noa Sendlhofer
 */

#ifndef PICO_MOTORS_MOTOR_DRIVER_H_
#define PICO_MOTORS_MOTOR_DRIVER_H_

#include <cstdint>

class Motor
{
public:
    Motor(uint8_t pin);

private:
    uint8_t pin;
};

#endif //PICO_MOTORS_MOTOR_DRIVER_H_
