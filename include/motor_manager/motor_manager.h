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

// ----------- CREEP -----------
#define US_PER_CM 146667
#define CREEP_SPEED 6000

// ----------- TURN -----------
#define TURN_SPEED 6000
#define US_PER_DEG_PER_SPEED 3.6

// ----------- PICKUP RIGHT -----------
#define PR_HOME_POS 167
#define PR_DOWN_POS 140
#define PR_GRAB_POS 112
#define PR_UP_POS 10

// ----------- PICKUP LEFT -----------
#define PL_HOME_POS (12 + 12)
#define PL_DOWN_POS (40 + 12)
#define PL_GRAB_POS (62 + 12)
#define PL_UP_POS (178 + 12)

// ----------- PICKUP -----------
#define PLIFT_PICKUP 5500
#define PLIFT_UNLOAD -8700
#define PLIFT_SET 5200

// ----------- UNLOAD -----------
#define UNLOAD_CLOSE_ANGLE 15
#define UNLOAD_OPEN_ANGLE 180

#define CURVE_SET_OFFSET 3

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
    void pickup(PickUpSide side) const;
    void moveMotors(int32_t position1, int32_t position2, int32_t speed) const;
    void moveServos(double angle1, double angle2, uint8_t speed1, uint8_t speed2) const;

    // mixer
    void setMixerDirection(MotorDirection direction) const;
    void setMixerSpeed(int32_t speed) const;

    // unload
    void driveToUnload() const;
    void driveFromUnload() const;
    void spinUnload() const;

    std::unique_ptr<Motor> drive_motor1;
    std::unique_ptr<Motor> drive_motor2;

    std::unique_ptr<Motor> mixer_motor;

    std::unique_ptr<CLMotor> crane_l_motor;
    std::unique_ptr<CLMotor> crane_r_motor;

    std::unique_ptr<Servo> crane_l_servo;
    std::unique_ptr<Servo> crane_r_servo;

    std::unique_ptr<Servo> unload_servo;
};


#endif //PICO_MOTORS_MOTOR_MANAGER_H_
