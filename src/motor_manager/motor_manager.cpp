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

    motor1->setSpeed(0);
    motor1->setDirection(STOP);

    motor2->setSpeed(0);
    motor2->setDirection(STOP);
}

void MotorManager::setSpeed(uint32_t target_speed, uint16_t rate)
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

void MotorManager::changeDirection()
{
    uint32_t old_speed = motor1->getCurrentSpeed();
    setSpeed(0, 200);
    motor1->changeDirection();
    motor2->changeDirection();
    setSpeed(old_speed, 200);
}

void MotorManager::turn(uint16_t degrees, Direction direction)
{
    uint8_t old_speed = motor1->getCurrentSpeed();
    setSpeed(0);

    if (direction == DIR_LEFT)
    {
        motor1->setDirection(FORWARD);
        motor2->setDirection(BACKWARD);
    }
    else if (direction == DIR_RIGHT)
    {
        motor1->setDirection(BACKWARD);
        motor2->setDirection(FORWARD);
    }
    else
        throw std::runtime_error("Invalid Direction");

    setSpeed(2000, 200);
    sleep_ms(MS_PER_DEG * degrees);

    setDirection(STOP);
    setSpeed(old_speed);
}

void MotorManager::setDirection(MotorDirection direction)
{
    motor1->setDirection(direction);
    motor2->setDirection(direction);
}

void MotorManager::creepDistance(uint16_t distance, MotorDirection direction)
{
    motor1->setSpeed(1000);
    motor2->setSpeed(1000);
    setDirection(direction);
    sleep_ms(distance * MS_PER_CM);
    setDirection(STOP);
}
