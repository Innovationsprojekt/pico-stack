/**
 * @file motor_manager.cpp
 * @author Noa Sendlhofer
 */

#include "motor_manager.h"
#include <stdlib.h>
#include <stdexcept>

MotorManager::MotorManager()
{
    motor1 = std::make_unique<Motor>(2,3);
    motor2 = std::make_unique<Motor>(4,5);

    crane_l_motor = std::make_unique<Motor>(6,7);
    crane_r_motor = std::make_unique<Motor>(8,9);

    crane_l_servo = std::make_unique<Servo>(10);
    crane_l_servo = std::make_unique<Servo>(12);
}

void MotorManager::setSpeed(int32_t set_speed) const
{
    motor1->setSpeed(set_speed);
    motor2->setSpeed(set_speed);
}

void MotorManager::rampSpeed(int32_t target_speed, uint16_t rate) const
{
    uint32_t current_speed = motor1->getCurrentSpeed();
    if (current_speed-target_speed == 0)
        return;

    double sleep_time = 1000.0/abs((int)current_speed-(int)target_speed) * rate;
    auto time_us = (uint32_t )sleep_time;

    if (target_speed < current_speed)
    {
        for (uint32_t i = current_speed; i > target_speed; i--)
        {
            motor1->setSpeed(i);
            motor2->setSpeed(i);
            sleep_us(time_us);
        }
    }
    else
    {
        for (uint32_t i = current_speed; i <= target_speed; i++)
        {
            motor1->setSpeed(i);
            motor2->setSpeed(i);
            sleep_us(time_us);
        }
    }
}

void MotorManager::changeDirection() const
{
    uint32_t old_speed = motor1->getCurrentSpeed();
    rampSpeed(0, 200);
    motor1->changeDirection();
    motor2->changeDirection();
    rampSpeed(old_speed, 200);
}

void MotorManager::turn(int16_t degrees, TurnDirection direction) const
{
    setSpeed(0);

    if (direction == LEFT)
    {
        motor1->setDirection(FORWARD);
        motor2->setDirection(BACKWARD);
    }
    else if (direction == RIGHT)
    {
        motor1->setDirection(BACKWARD);
        motor2->setDirection(FORWARD);
    }
    else
        throw std::runtime_error("Invalid Direction");

    rampSpeed(TURN_SPEED, 200);
    sleep_us(uint32_t(US_PER_DEG_PER_SPEED * degrees * TURN_SPEED));

    setDirection(STOP);
}

void MotorManager::setDirection(MotorDirection direction) const
{
    motor1->setDirection(direction);
    motor2->setDirection(direction);
}

void MotorManager::creepDistance(uint16_t distance, MotorDirection direction) const
{
    setSpeed(CREEP_SPEED);
    setDirection(direction);
    sleep_ms(distance * MS_PER_CM);
    setDirection(STOP);
}