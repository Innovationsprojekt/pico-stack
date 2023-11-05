/**
 * @file stepper_driver.h
 * @author Noa Sendlhofer
 */

#ifndef PICO_MOTORS_MOTOR_DRIVER_H_
#define PICO_MOTORS_MOTOR_DRIVER_H_

#include <cstdint>
#include <hardware/pwm.h>
#include <pico/stdlib.h>

#define CLOCK_SPEED 1.25e+8

enum MotorDirection
{
    FORWARD = 0,
    BACKWARD = 1,
    STOP = 2,
};

class MotorDriver
{
protected:
    explicit MotorDriver(uint8_t step_pin, uint8_t clk_div, uint16_t wrap);

    void writePWM(int32_t pwm_width);
    uint32_t getCurrentPWM() const;

private:
    void _setEnabled(bool state);

    const uint8_t step_pin;

    uint slice_num;
    uint pwm_channel;

    uint8_t clk_div;
    uint16_t wrap;

    bool enabled = false;
    uint32_t current_pwm = 0;

};

#endif //PICO_MOTORS_MOTOR_DRIVER_H_
