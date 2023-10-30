/**
 * @file controller.cpp
 * @author Noa Sendlhofer
 */

#include "controller.h"

#include <utility>
#include <valarray>
#include <stdexcept>

Controller::Controller(std::shared_ptr<MotorManager> motor_manager, std::shared_ptr<SensorManager> sensor_manager)
    : motor_manager(std::move(motor_manager))
    , sensor_manager(std::move(sensor_manager))
{
    this->motor_manager->setDirection(BACKWARD);
}

void Controller::spin(double dt)
{
    _driveClosedLoop(dt);
    _checkGamePosition();
}

void Controller::_driveClosedLoop(double dt)
{
    int32_t position_error = sensor_manager->getHorizontalPosition(SENSOR_ROW_BACK);

    double Pout = _Kp * position_error;

    double derivative = (position_error - _last_error) / dt;
    double Dout = _Kd * derivative;

    double output = (Pout + Dout);

    _last_error = position_error;

    int32_t motor1_out = _base_speed - (int32_t)output / 10;
    int32_t motor2_out = _base_speed + (int32_t)output / 10;

    printf("pos_err: %li, Pout: %.2f, Dout %.2f, out %.2f, M1 %li, M2 %li\n\r", position_error, Pout, Dout, output, motor1_out, motor2_out);

    motor_manager->motor1->setSpeed(motor1_out);
    motor_manager->motor2->setSpeed(motor2_out);
}

void Controller::_alignTangential(Direction turn_direction)
{
    motor_manager->motor1->setSpeed(800);
    motor_manager->motor2->setSpeed(800);

    printf("Sensor Row FRONT: %li\n\r", sensor_manager->getHorizontalPosition(SENSOR_ROW_FRONT));
    printf("Sensor Row BACK: %li\n\r", sensor_manager->getHorizontalPosition(SENSOR_ROW_BACK));

    if (turn_direction == DIR_LEFT)
    {
        motor_manager->motor1->setDirection(FORWARD);
        motor_manager->motor2->setDirection(BACKWARD);
    } else
    {
        motor_manager->motor1->setDirection(BACKWARD);
        motor_manager->motor2->setDirection(FORWARD);
    }

    while(abs(sensor_manager->getHorizontalPosition(SENSOR_ROW_FRONT) - sensor_manager->getHorizontalPosition(SENSOR_ROW_BACK)) > OFFSET_ERROR_TANGENTIAL)
        sleep_ms(1);

    motor_manager->setDirection(STOP);

    printf("Sensor Row FRONT: %li\n\r", sensor_manager->getHorizontalPosition(SENSOR_ROW_FRONT));
    printf("Sensor Row BACK: %li\n\r", sensor_manager->getHorizontalPosition(SENSOR_ROW_BACK));
}

void Controller::_alignHorizontal()
{
    int32_t s_c1 = sensor_manager->readSensor(SENSOR_C1);
    //int32_t s_c2 = sensor_manager->readSensor(SENSOR_C2);
    printf("Sensor C1: %li\n\r", s_c1);

    motor_manager->motor1->setSpeed(1000);
    motor_manager->motor2->setSpeed(1000);

    if (s_c1 < ON_LINE)
        motor_manager->setDirection(FORWARD);
    else
        motor_manager->setDirection(BACKWARD);

    while (abs(sensor_manager->readSensor(SENSOR_C1) - ON_LINE) > OFFSET_ERROR_HORIZONTAL)
        sleep_ms(2);

    motor_manager->setDirection(STOP);
    s_c1 = sensor_manager->readSensor(SENSOR_C1);
    printf("Sensor C1: %li\n\r", s_c1);
}

void Controller::_checkGamePosition()
{
    GamePosition current_pos = __game_configuration.at(_current_position);

    if (__readLineSensor(current_pos) < BLACK)
    {
        motor_manager->setDirection(STOP);

        switch (current_pos)
        {
            case STRAIGHT:
                for (int i = 0; i < 2; i++)
                {
                    sleep_ms(500);
                    _alignTangential(__findTurnDirection());
                    sleep_ms(500);
                    _alignHorizontal();
                    sleep_ms(500);
                    _alignTangential(__findTurnDirection());
                }

                sleep_ms(2000);

                motor_manager->motor1->setSpeed(1000);
                motor_manager->motor2->setSpeed(1000);
                motor_manager->setDirection(BACKWARD);
                sleep_ms(700);
                break;
            case TURN_RIGHT:
                break;
            case TURN_LEFT:
                for (int i = 0; i < 2; i++)
                {
                    sleep_ms(500);
                    _alignTangential(DIR_LEFT);
                    sleep_ms(500);
                    _alignHorizontal();
                    sleep_ms(500);
                    _alignTangential(__findTurnDirection());
                }

                sleep_ms(2000);

                motor_manager->motor1->setSpeed(1000);
                motor_manager->motor2->setSpeed(1000);
                motor_manager->setDirection(FORWARD);
                sleep_ms(500);
                motor_manager->setDirection(STOP);
                sleep_ms(500);
                motor_manager->turn(20, DIR_RIGHT);
                sleep_ms(500);
                motor_manager->motor1->setSpeed(1000);
                motor_manager->motor2->setSpeed(1000);
                motor_manager->setDirection(BACKWARD);
                sleep_ms(600);
                break;
        }
        _current_position++;
    }
}

int32_t Controller::__readLineSensor(GamePosition current_pos)
{
    switch (current_pos)
    {
        case STRAIGHT:
            return sensor_manager->readSensor(SENSOR_C1);
        case TURN_LEFT:
            return sensor_manager->readSensor(SENSOR_C1);
        case TURN_RIGHT:
            return sensor_manager->readSensor(SENSOR_C2);
    }
}

Direction Controller::__findTurnDirection()
{
    if (sensor_manager->getHorizontalPosition(SENSOR_ROW_FRONT) < sensor_manager->getHorizontalPosition(SENSOR_ROW_BACK))
        return DIR_LEFT;
    else
        return DIR_RIGHT;
}
