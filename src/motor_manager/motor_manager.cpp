/**
 * @file motor_manager.cpp
 * @author Noa Sendlhofer
 */

#include "motor_manager.h"
#include <stdlib.h>

MotorManager::MotorManager()
{
    motor1 = std::make_unique<Motor>(0,1);
    motor2 = std::make_unique<Motor>(2,3);

    motor1->setSpeed(0);
    motor1->setDirection(STOP);

    motor2->setSpeed(0);
    motor2->setDirection(STOP);
}

void MotorManager::setSpeed(uint8_t target_speed, uint16_t rate)
{
    uint8_t current_speed = motor1->getCurrentSpeed();
    if (current_speed-target_speed == 0)
        return;
    uint16_t sleep_time = rate/abs(current_speed-target_speed);

    if (target_speed < current_speed)
    {
        for (uint8_t i = current_speed; i > target_speed; i--)
        {
            motor1->setSpeed(i);
            motor2->setSpeed(i);
            sleep_ms(sleep_time);
        }
    }
    else
    {
        for (uint8_t i = current_speed; i <= target_speed; i++)
        {
            motor1->setSpeed(i);
            motor2->setSpeed(i);
            sleep_ms(sleep_time);
        }
    }
}

void MotorManager::changeDirection()
{
    uint8_t old_speed = motor1->getCurrentSpeed();
    setSpeed(0, 200);
    motor1->changeDirection();
    motor2->changeDirection();
    setSpeed(old_speed, 200);
}

void MotorManager::turn(uint16_t degrees, bool direction)
{
    uint8_t old_speed = motor1->getCurrentSpeed();
    MotorDirection old_dir = motor1->getCurrentDirection();
    setSpeed(0);

    if (direction)
    {
        motor1->setDirection(FORWARD);
        motor2->setDirection(BACKWARD);
    }
    else
    {
        motor1->setDirection(BACKWARD);
        motor2->setDirection(FORWARD);
    }

    setSpeed(20, 200);
    sleep_ms(137000/degrees);
    setSpeed(0, 100);
    motor1->setDirection(old_dir);
    motor2->setDirection(old_dir);
    setSpeed(old_speed);
}

void MotorManager::setDirection(MotorDirection direction)
{
    motor1->setDirection(direction);
    motor2->setDirection(direction);
}

void MotorManager::omniWheels(Direction direction)
{

}
