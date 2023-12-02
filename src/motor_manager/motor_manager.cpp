/**
 * @file motor_manager.cpp
 * @author Noa Sendlhofer
 */

#include "motor_manager.h"
#include <stdlib.h>
#include <stdexcept>

MotorManager::MotorManager()
{
    drive_motor2 = std::make_unique<Motor>(0, 1);
    drive_motor1 = std::make_unique<Motor>(2, 3);

    mixer_motor = std::make_unique<Motor>(10, 11);

    std::shared_ptr<RotaryEncoder> encoder1 = std::make_shared<RotaryEncoder>(18, 19);
    std::shared_ptr<RotaryEncoder1> encoder2 = std::make_shared<RotaryEncoder1>(20, 21);

    crane_r_motor = std::make_unique<CLMotor>(8,9, encoder1);
    crane_l_motor = std::make_unique<CLMotor>(6,7,encoder2);

    crane_l_servo = std::make_unique<Servo>(12);
    crane_r_servo = std::make_unique<Servo>(14);

    unload_servo = std::make_unique<Servo>(13);
}

void MotorManager::setSpeed(int32_t set_speed) const
{
    drive_motor1->setSpeed(set_speed);
    drive_motor2->setSpeed(set_speed);
}

void MotorManager::rampSpeed(int32_t target_speed, uint16_t rate) const
{
    uint32_t current_speed = drive_motor1->getCurrentSpeed();
    if (current_speed-target_speed == 0)
        return;

    double sleep_time = 1000.0/abs((int)current_speed-(int)target_speed) * rate;
    auto time_us = (uint32_t )sleep_time;

    if (target_speed < current_speed)
    {
        for (uint32_t i = current_speed; i > target_speed; i--)
        {
            drive_motor1->setSpeed(i);
            drive_motor2->setSpeed(i);
            sleep_us(time_us);
        }
    }
    else
    {
        for (uint32_t i = current_speed; i <= target_speed; i++)
        {
            drive_motor1->setSpeed(i);
            drive_motor2->setSpeed(i);
            sleep_us(time_us);
        }
    }
}

void MotorManager::changeDirection() const
{
    uint32_t old_speed = drive_motor1->getCurrentSpeed();
    rampSpeed(0, 200);
    drive_motor1->changeDirection();
    drive_motor2->changeDirection();
    rampSpeed(old_speed, 200);
}

void MotorManager::turn(int16_t degrees, TurnDirection direction) const
{
    setSpeed(0);

    if (direction == LEFT)
    {
        drive_motor1->setDirection(FORWARD);
        drive_motor2->setDirection(BACKWARD);
    }
    else if (direction == RIGHT)
    {
        drive_motor1->setDirection(BACKWARD);
        drive_motor2->setDirection(FORWARD);
    }
    else
        throw std::runtime_error("Invalid Direction");

    rampSpeed(TURN_SPEED, 200);
    sleep_us(uint32_t(US_PER_DEG_PER_SPEED * degrees * TURN_SPEED));

    setDirection(STOP);
}

void MotorManager::setDirection(MotorDirection direction) const
{
    drive_motor1->setDirection(direction);
    drive_motor2->setDirection(direction);
}

void MotorManager::creepDistance(double distance, MotorDirection direction) const
{
    setSpeed(CREEP_SPEED);
    setDirection(direction);
    sleep_us(int64_t(distance * US_PER_CM));
    setDirection(STOP);
}

void MotorManager::setMixerDirection(MotorDirection direction) const
{
    mixer_motor->setDirection(direction);
}

void MotorManager::setMixerSpeed(int32_t speed) const
{
    mixer_motor->setSpeed(speed);
}

void MotorManager::pickup(PickUpSide side) const
{
    switch (side)
    {
        case PICKUP_RIGHT:
            crane_r_servo->setAngle(PR_DOWN_POS);
            crane_r_motor->setPosition(PLIFT_PICKUP, 10000);
            crane_r_servo->setAngle(PR_GRAB_POS);
            sleep_ms(200);
            crane_r_motor->setPosition(PLIFT_UNLOAD, 10000);
            crane_r_servo->setAngle(PR_UP_POS, 3);
            sleep_ms(800);
            crane_r_servo->setAngle(PR_GRAB_POS + 10);
            crane_r_motor->setPosition(PLIFT_SET, 10000);
            crane_r_servo->setAngle(PR_DOWN_POS);
            crane_r_motor->setPosition(0, 10000);
            homePickup(PICKUP_RIGHT);
            break;

        case PICKUP_RIGHT_SHIELD:
            crane_r_servo->setAngle(PR_DOWN_POS);
            crane_r_motor->setPosition(PLIFT_PICKUP, 10000);
            crane_r_servo->setAngle(PR_GRAB_POS);
            sleep_ms(200);
            moveMotors(PLIFT_UNLOAD, PLIFT_UNLOAD, 10000);
            moveServos(PR_UP_POS, PL_UP_POS, 3, 5);
            sleep_ms(800);
            moveServos(PR_GRAB_POS + 10, PL_HOME_POS, 5, 5);
            moveMotors(PLIFT_SET, 0, 10000);
            crane_r_servo->setAngle(PR_DOWN_POS);
            crane_r_motor->setPosition(0, 10000);
            homePickup(PICKUP_RIGHT);
            break;

        case PICKUP_RIGHT_CURVE:
            creepDistance(0.75, BACKWARD);

            crane_r_servo->setAngle(PR_DOWN_POS);
            crane_r_motor->setPosition(PLIFT_PICKUP, 10000);
            crane_r_servo->setAngle(PR_GRAB_POS - 5);
            sleep_ms(200);
            crane_r_motor->setPosition(PLIFT_UNLOAD, 10000);
            crane_r_servo->setAngle(PR_UP_POS, 3);
            sleep_ms(800);
            crane_r_servo->setAngle(PR_GRAB_POS + 10 - CURVE_SET_OFFSET);
            crane_r_motor->setPosition(PLIFT_SET + 500, 10000);
            crane_r_servo->setAngle(PR_DOWN_POS);
            crane_r_motor->setPosition(0, 10000);
            homePickup(PICKUP_RIGHT);
            break;

        case PICKUP_LEFT:
            crane_l_servo->setAngle(PL_DOWN_POS);
            crane_l_motor->setPosition(PLIFT_PICKUP, 10000);
            crane_l_servo->setAngle(PL_GRAB_POS);
            sleep_ms(200);
            crane_l_motor->setPosition(PLIFT_UNLOAD, 10000);
            crane_l_servo->setAngle(PL_UP_POS, 3);
            sleep_ms(800);
            crane_l_servo->setAngle(PL_GRAB_POS - 10);
            crane_l_motor->setPosition(PLIFT_SET, 10000);
            crane_l_servo->setAngle(PL_DOWN_POS);
            crane_l_motor->setPosition(0, 10000);
            homePickup(PICKUP_LEFT);
            break;

        case PICKUP_LEFT_SHIELD:
            crane_l_servo->setAngle(PL_DOWN_POS);
            crane_l_motor->setPosition(PLIFT_PICKUP, 10000);
            crane_l_servo->setAngle(PL_GRAB_POS);
            sleep_ms(200);
            moveMotors(PLIFT_UNLOAD, PLIFT_UNLOAD, 10000);
            moveServos(PR_UP_POS, PL_UP_POS, 5, 3);
            sleep_ms(800);
            moveServos(PR_HOME_POS, PL_GRAB_POS - 10, 5, 5);
            moveMotors(0, PLIFT_SET, 10000);
            crane_l_servo->setAngle(PL_DOWN_POS);
            crane_l_motor->setPosition(0, 10000);
            homePickup(PICKUP_LEFT);
            break;

        case PICKUP_LEFT_CURVE:
            crane_l_servo->setAngle(PL_DOWN_POS);
            crane_l_motor->setPosition(PLIFT_PICKUP, 10000);
            crane_l_servo->setAngle(PL_GRAB_POS + 5);
            sleep_ms(200);
            crane_l_motor->setPosition(PLIFT_UNLOAD, 10000);
            crane_l_servo->setAngle(PL_UP_POS, 3);
            sleep_ms(800);
            crane_l_servo->setAngle(PL_GRAB_POS - 10 + CURVE_SET_OFFSET);
            crane_l_motor->setPosition(PLIFT_SET + 500, 10000);
            crane_l_servo->setAngle(PL_DOWN_POS);
            crane_l_motor->setPosition(0, 10000);
            homePickup(PICKUP_LEFT);
            break;
    }
}

void MotorManager::homePickup(PickUpSide side) const
{
    switch (side)
    {
        case PICKUP_LEFT:
            crane_l_servo->setAngle(PL_HOME_POS);
            break;
        case PICKUP_RIGHT:
            crane_r_servo->setAngle(PR_HOME_POS);
            break;
    }
}

void MotorManager::moveMotors(int32_t position1, int32_t position2, int32_t speed) const
{
    bool left_goal = false;
    bool right_goal = false;

    while (true)
    {
        if (!right_goal)
            right_goal = crane_r_motor->spinSetPosition(position1, speed);

        if (!left_goal)
            left_goal = crane_l_motor->spinSetPosition(position2, speed);

        if (left_goal && right_goal)
            return;

        sleep_ms(2);
    }
}

void MotorManager::moveServos(double angle1, double angle2, uint8_t speed1, uint8_t speed2) const
{
    bool left_goal = false;
    bool right_goal = false;

    while (true)
    {
        if (!right_goal)
            right_goal = crane_r_servo->spinSetAngle(angle1, speed1);

        if (!left_goal)
            left_goal = crane_l_servo->spinSetAngle(angle2, speed2);

        if (left_goal && right_goal)
            return;

        sleep_ms(SERVO_SPEED);
    }
}
