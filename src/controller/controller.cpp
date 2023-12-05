/**
 * @file controller.cpp
 * @author Noa Sendlhofer
 */

#include "controller.h"

#include <valarray>
#include <stdexcept>

Controller::Controller()
{
    _motor_manager = std::make_shared<MotorManager>();
    _sensor_manager = std::make_shared<SensorManager>(_motor_manager);

    _motor_manager->homePickup(PICKUP_LEFT);
    _motor_manager->homePickup(PICKUP_RIGHT);

    _motor_manager->setMixerDirection(STOP);

    _motor_manager->unload_servo->setAngle(0);
    /*
    _sensor_manager->calibrate();

    while (true)
    {
        printf("Old Pos Error: %li, Full Pos Error: %li\n\r", _sensor_manager->getHorizontalPosition(SENSOR_ROW_BACK) -
                                                              _sensor_manager->getHorizontalPosition(SENSOR_ROW_FRONT),
               _sensor_manager->getFullHorizontalPosition(SENSOR_ROW_BACK) -
               _sensor_manager->getFullHorizontalPosition(SENSOR_ROW_FRONT));
        sleep_ms(100);
    }
     */
}

void Controller::spin(double dt)
{
#ifdef ENABLE_SENSOR_CALIB
    if (_sensor_manager->isCalibrated())
    {
        /*
        printf("FLO %li, FLI %li, FRI %li, FRO %li\n\r"
               "CLO %li, CLI %li, CRI %li, CRO %li\n\r"
               "BLO %li, BLI %li, BRI %li, BRO %li\n\r",
               _sensor_manager->readSensor(SENSOR_FLO),
               _sensor_manager->readSensor(SENSOR_FLI),
               _sensor_manager->readSensor(SENSOR_FRI),
               _sensor_manager->readSensor(SENSOR_FRO),
               _sensor_manager->readSensor(SENSOR_CLO),
               _sensor_manager->readSensor(SENSOR_CLI),
               _sensor_manager->readSensor(SENSOR_CRI),
               _sensor_manager->readSensor(SENSOR_CRO),
               _sensor_manager->readSensor(SENSOR_BLO),
               _sensor_manager->readSensor(SENSOR_BLI),
               _sensor_manager->readSensor(SENSOR_BRI),
               _sensor_manager->readSensor(SENSOR_BRO));
               */
        printf("POSF %li, POSC %li, POSB %li\n\r",
               _sensor_manager->getHorizontalPosition(SENSOR_ROW_FRONT),
               _sensor_manager->getHorizontalPosition(SENSOR_ROW_CENTER),
               _sensor_manager->getHorizontalPosition(SENSOR_ROW_BACK));
        return;
    }
#endif

    if (_enable_drive)
        _driveClosedLoop(dt);

    if (_enable_detection)
        _detectLine();
}

void Controller::_driveClosedLoop(double dt)
{
    int32_t position_error = _sensor_manager->getHorizontalPosition(SENSOR_ROW_FRONT);

    double Pout = _drive_kp * position_error;

    double derivative = (position_error - _last_error_drive) / dt;
    double Dout = _drive_kd * derivative;

    double output = Pout + Dout;

    _last_error_drive = position_error;

    int32_t motor1_out = _drive_speed + (int32_t)output / 4;
    int32_t motor2_out = _drive_speed - (int32_t)output / 4;

    _motor_manager->drive_motor1->setSpeed(motor1_out);
    _motor_manager->drive_motor2->setSpeed(motor2_out);
}

void Controller::_alignTangentialPID()
{
    _motor_manager->setSpeed(0);

    _integral_tan = 0;
    _last_error_tan = 0;

    int32_t position_error = _sensor_manager->getHorizontalPosition(SENSOR_ROW_BACK) - _sensor_manager->getHorizontalPosition(SENSOR_ROW_FRONT);

    while(abs(position_error) > _offset)
    {
        position_error = _sensor_manager->getHorizontalPosition(SENSOR_ROW_BACK) - _sensor_manager->getHorizontalPosition(SENSOR_ROW_FRONT);
        __spinAlignTangential(1000.0 / ALIGN_TAN_FREQ, position_error);
        sleep_ms(1000 / ALIGN_TAN_FREQ);
    }
    _motor_manager->setDirection(STOP);
}

void Controller::_alignFullTangentialPID()
{
    _motor_manager->setSpeed(0);

    _integral_tan = 0;
    _last_error_tan = 0;

    int32_t position_error = _sensor_manager->getFullHorizontalPosition(SENSOR_ROW_BACK) - _sensor_manager->getFullHorizontalPosition(SENSOR_ROW_FRONT);

    for (int i = 0; i < 3; i++)
    {
        while(abs(position_error) > _offset)
        {
            position_error = _sensor_manager->getFullHorizontalPosition(SENSOR_ROW_BACK) - _sensor_manager->getFullHorizontalPosition(SENSOR_ROW_FRONT);
            __spinAlignTangential(1000.0 / ALIGN_TAN_FREQ, position_error);
            sleep_ms(1000 / ALIGN_TAN_FREQ);
        }
    }
    _motor_manager->setDirection(STOP);
}

void Controller::__spinAlignTangential(double dt, int32_t position_error)
{
    double Pout = _align_kp * position_error;

    _integral_tan += int32_t(dt * position_error);
    double Iout = _align_ki * _integral_tan;

    double derivative = (position_error - _last_error_tan) / dt;
    double Dout = _align_kd * derivative;

    double output = Pout + Dout + Iout;

    _last_error_tan = position_error;

    int32_t motor_out = (int32_t)output / 4;

    if (motor_out > 0)
    {
        _motor_manager->setSpeed(motor_out);
        _motor_manager->drive_motor1->setDirection(FORWARD);
        _motor_manager->drive_motor2->setDirection(BACKWARD);
    } else
    {
        _motor_manager->setSpeed(- motor_out);
        _motor_manager->drive_motor1->setDirection(BACKWARD);
        _motor_manager->drive_motor2->setDirection(FORWARD);
    }

    printf("pos_err: %li, Pout: %.2f, Dout %.2f, out %.2f\n\r", position_error, Pout, Dout, output);
}

void Controller::_alignHorizontal()
{
    int32_t s_c1 = _sensor_manager->readSensor(_line_sensor);
    printf("Sensor C1: %li\n\r", s_c1);

    _motor_manager->drive_motor1->setSpeed(ALIGN_HOR_BASE_SPEED);
    _motor_manager->drive_motor2->setSpeed(ALIGN_HOR_BASE_SPEED);

    if (s_c1 < ON_LINE)
        _motor_manager->setDirection(FORWARD);
    else
        _motor_manager->setDirection(BACKWARD);

    while (abs(_sensor_manager->readSensor(_line_sensor) - ON_LINE) > OFFSET_ERROR_HORIZONTAL)
        sleep_ms(2);

    _motor_manager->setDirection(STOP);
    s_c1 = _sensor_manager->readSensor(_line_sensor);
    printf("Sensor C1: %li\n\r", s_c1);
}

void Controller::_detectLine()
{
    if (_sensor_manager->readSensor(_line_sensor) < BLACK)
    {
        _motor_manager->setDirection(STOP);
        _enable_drive = false;
        _enable_detection = false;

        _executor->notify(NOTIFY_LINE);
    }
}

void Controller::start()
{
    _executor->notify(NOTIFY_READY);
}

void Controller::_pickup(PickUpSide side) const
{
    _motor_manager->pickup(side);
}

void Controller::_unload()
{
    _alignHorizontal();
    _alignTangentialPID();

    //GATE
    _motor_manager->creepDistance(5.5, FORWARD);

    _motor_manager->drive_motor2->setDirection(STOP);
    _motor_manager->drive_motor1->setSpeed(8000);
    _motor_manager->drive_motor1->setDirection(FORWARD);
    sleep_ms(800);
    _motor_manager->drive_motor1->setDirection(STOP);

    //CONT1
    _motor_manager->creepDistance(8, BACKWARD);

    _motor_manager->drive_motor2->setDirection(STOP);
    _motor_manager->drive_motor1->setSpeed(8000);
    _motor_manager->drive_motor1->setDirection(FORWARD);
    sleep_ms(1000);
    _motor_manager->drive_motor1->setDirection(STOP);

    //FRONT
    _motor_manager->creepDistance(8, BACKWARD);

    _motor_manager->drive_motor1->setDirection(STOP);
    _motor_manager->drive_motor2->setSpeed(8000);
    _motor_manager->drive_motor2->setDirection(BACKWARD);
    sleep_ms(1000);
    _motor_manager->drive_motor2->setDirection(STOP);

    //SLIGHT BACK
    _motor_manager->creepDistance(5, FORWARD);

    //STRAIGHT
    _motor_manager->drive_motor2->setDirection(STOP);
    _motor_manager->drive_motor1->setSpeed(8000);
    _motor_manager->drive_motor1->setDirection(FORWARD);
    sleep_ms(100);
    _motor_manager->drive_motor1->setDirection(STOP);


    //CONT
    _motor_manager->creepDistance(3, FORWARD);
    _motor_manager->drive_motor1->setDirection(STOP);

    _motor_manager->unload_servo->setAngle(UNLOAD_OPEN);

    _motor_manager->setSpeed(10000);
    for (int n = 0; n<_unload_counter/5; n++)
    {
        _motor_manager->unload_servo->setAngle(UNLOAD_OPEN - 50);
        for (int i = 0; i<5; i++)
        {
            _motor_manager->setDirection(BACKWARD);
            sleep_ms(150);
            _motor_manager->setDirection(FORWARD);
            sleep_ms(150);
        }
        _motor_manager->unload_servo->setAngle(UNLOAD_OPEN);
    }

    _motor_manager->setDirection(STOP);

    _motor_manager->unload_servo->setAngle(UNLOAD_CLOSE);

    _motor_manager->creepDistance(6, BACKWARD);
    _motor_manager->drive_motor1->setDirection(STOP);

    _motor_manager->turn(20, RIGHT);

    _motor_manager->drive_motor2->setSpeed(8000);
    _motor_manager->drive_motor2->setDirection(FORWARD);
    sleep_ms(2400);
    _motor_manager->drive_motor2->setDirection(STOP);

    _motor_manager->creepDistance(12.5, BACKWARD);
    _motor_manager->drive_motor1->setDirection(STOP);

    _motor_manager->turn(10, LEFT);

#ifdef GAME_PLAN_UNLOAD
    _unload_counter += 30;
#endif
}