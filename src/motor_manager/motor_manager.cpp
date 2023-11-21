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

    mixer_speed = std::make_unique<Motor>(6, 7);

    std::shared_ptr<RotaryEncoder> encoder1 = std::make_shared<RotaryEncoder>(18, 19);
    std::shared_ptr<RotaryEncoder1> encoder2 = std::make_shared<RotaryEncoder1>(20, 21);

    crane_r_motor = std::make_unique<CLMotor>(4,5, encoder1);
    crane_l_motor = std::make_unique<CLMotor>(8,9,encoder2);

    crane_l_servo = std::make_unique<Servo>(12);
    crane_r_servo = std::make_unique<Servo>(14);
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
    sleep_ms(int16_t(distance * MS_PER_CM));
    setDirection(STOP);
}

void MotorManager::setMixerDirection(MotorDirection direction)
{
    mixer_speed->setDirection(direction);
}

void MotorManager::setMixerSpeed(int32_t speed)
{
    mixer_speed->setSpeed(speed);
}

void MotorManager::pickup(PickUpSide side)
{
    switch (side)
    {
        case PICKUP_RIGHT:
            crane_r_servo->setAngle(140);
            crane_r_motor->setPosition(5500, 7000);
            crane_r_servo->setAngle(115);
            sleep_ms(200);
            crane_r_motor->setPosition(-7000, 7000);
            crane_r_servo->setAngle(17);
            sleep_ms(800);
            crane_r_servo->setAngle(120);
            crane_r_motor->setPosition(5000, 7000);
            crane_r_servo->setAngle(140);
            crane_r_motor->setPosition(0, 7000);
            homePickup(PICKUP_RIGHT);
            break;
        case PICKUP_LEFT:
            crane_l_servo->setAngle(40);
            crane_l_motor->setPosition(5500, 7000);
            crane_l_servo->setAngle(60);
            sleep_ms(200);
            crane_l_motor->setPosition(-7000, 7000);
            crane_l_servo->setAngle(160);
            sleep_ms(800);
            crane_l_servo->setAngle(55);
            crane_l_motor->setPosition(5000, 7000);
            crane_l_servo->setAngle(40);
            crane_l_motor->setPosition(0, 7000);
            homePickup(PICKUP_LEFT);
            break;
    }
}

void MotorManager::homePickup(PickUpSide side) const
{
    switch (side)
    {
        case PICKUP_LEFT:
            crane_l_servo->setAngle(0);
            break;
        case PICKUP_RIGHT:
            crane_r_servo->setAngle(175);
            break;
    }
}
