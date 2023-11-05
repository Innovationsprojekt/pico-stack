/**
 * @file motor_manager.h
 * @author Noa Sendlhofer
 */

#ifndef PICO_MOTORS_MOTOR_MANAGER_H_
#define PICO_MOTORS_MOTOR_MANAGER_H_

#include <memory>
#include "motor.h"
#include "servo.h"

#define MS_PER_CM 260
#define CREEP_SPEED 4000

#define TURN_SPEED 6000
#define US_PER_DEG_PER_SPEED 4.3

enum Direction
{
    DIR_FORWARD,
    DIR_BACKWARD,
    DIR_LEFT,
    DIR_RIGHT,
};

class MotorManager
{
public:
    MotorManager();
    void rampSpeed(int32_t target_speed, uint16_t rate = 200) const;
    void setSpeed(int32_t set_speed) const;
    void changeDirection() const;
    void setDirection(MotorDirection direction) const;
    void turn(int16_t degrees, Direction direction) const;
    void creepDistance(uint16_t distance, MotorDirection direction) const;

    std::unique_ptr<Motor> motor1;
    std::unique_ptr<Motor> motor2;

    std::unique_ptr<Motor> crane_l_motor;
    std::unique_ptr<Motor> crane_r_motor;

    std::unique_ptr<Servo> crane_l_servo;
    std::unique_ptr<Servo> crane_r_servo;
};


#endif //PICO_MOTORS_MOTOR_MANAGER_H_
