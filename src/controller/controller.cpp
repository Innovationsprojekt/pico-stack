/**
 * @file controller.cpp
 * @author Noa Sendlhofer
 */

#include "controller.h"

#include <utility>
#include <valarray>

Controller::Controller(std::shared_ptr<MotorManager> motor_manager, std::shared_ptr<SensorManager> sensor_manager)
    : motor_manager(std::move(motor_manager))
    , sensor_manager(std::move(sensor_manager))
{
}

void Controller::spin(double dt)
{

    _driveClosedLoop(dt);

    if (sensor_manager->readSensor(SENSOR_C1) < BLACK)
    {
        motor_manager->setDirection(STOP);
        _alignTangential();
    }
}

void Controller::_driveClosedLoop(double dt)
{
    int32_t position_error = sensor_manager->getHorizontalPosition(SENSOR_ROW_FRONT);

    double Pout = _Kp * position_error;

    double derivative = (position_error - _last_error) / dt;
    double Dout = _Kd * derivative;

    double output = (Pout + Dout);

    _last_error = position_error;

    motor_manager->motor1->setSpeed(base_speed-(int32_t)output/10);
    motor_manager->motor2->setSpeed(base_speed+(int32_t)output/10);
}

void Controller::_alignTangential()
{
    motor_manager->motor1->setSpeed(500);
    motor_manager->motor2->setSpeed(500);

    if (sensor_manager->getHorizontalPosition(SENSOR_ROW_FRONT) > sensor_manager->getHorizontalPosition(SENSOR_ROW_BACK))
    {
        motor_manager->motor1->setDirection(FORWARD);
        motor_manager->motor2->setDirection(BACKWARD);
    } else
    {
        motor_manager->motor1->setDirection(BACKWARD);
        motor_manager->motor2->setDirection(FORWARD);
    }

    while(abs(sensor_manager->getHorizontalPosition(SENSOR_ROW_FRONT) - sensor_manager->getHorizontalPosition(SENSOR_ROW_BACK)) > OFFSET_ERROR)
        sleep_ms(5);

    motor_manager->setDirection(STOP);
}
