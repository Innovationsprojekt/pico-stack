/**
 * @file motor_manager.h
 * @author Noa Sendlhofer
 */

#ifndef PICO_MOTORS_MOTOR_MANAGER_H_
#define PICO_MOTORS_MOTOR_MANAGER_H_

#include <memory>
#include "motor.h"
#include "servo.h"
#include "cl_motor.h"
#include "enum_definitions.h"

#define MS_PER_CM 220
#define CREEP_SPEED 4000

#define TURN_SPEED 6000
#define US_PER_DEG_PER_SPEED 3.6

class MotorManager
{
public:
    MotorManager();

    // drive
    void rampSpeed(int32_t target_speed, uint16_t rate = 200) const;
    void setSpeed(int32_t set_speed) const;
    void changeDirection() const;
    void setDirection(MotorDirection direction) const;
    void turn(int16_t degrees, TurnDirection direction) const;
    void creepDistance(double distance, MotorDirection direction) const;

    // pickup
    void homePickup(PickUpSide side) const;
    void pickup(PickUpSide side);

    // mixer
    void setMixerDirection(MotorDirection direction);
    void setMixerSpeed(int32_t speed);

    std::unique_ptr<Motor> drive_motor1;
    std::unique_ptr<Motor> drive_motor2;

    std::unique_ptr<Motor> mixer_speed;

    std::unique_ptr<CLMotor> crane_l_motor;
    std::unique_ptr<CLMotor> crane_r_motor;

    std::unique_ptr<Servo> crane_l_servo;
    std::unique_ptr<Servo> crane_r_servo;
};


#endif //PICO_MOTORS_MOTOR_MANAGER_H_
