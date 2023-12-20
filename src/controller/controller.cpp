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
    _motor_manager->setMixerSpeed(MIXER_SPEED);
    _motor_manager->setMixerDirection(BACKWARD);

    sleep_ms(1000);

    while (true)
    {
        for (int i = 0; i < 2; i++)
        {
            _pickup(PICKUP_LEFT);
            sleep_ms(5500);
            _pickup(PICKUP_RIGHT);
            sleep_ms(5500);
        }
        sleep_ms(2000);

        _motor_manager->setMixerDirection(FORWARD);

        _motor_manager->setSpeed(10000);
        for (int i = 0; i<5; i++)
        {
            _motor_manager->setDirection(FORWARD);
            sleep_ms(150);
            _motor_manager->setDirection(BACKWARD);
            sleep_ms(150);
        }
        _motor_manager->setDirection(STOP);

        _motor_manager->setMixerDirection(BACKWARD);
    }
     */
    /*
    _sensor_manager->calibrate();

    while (true)
    {
        printf("CLO %li, CRI %li\n\r", _sensor_manager->readSensor(SENSOR_CLO), _sensor_manager->readSensor(SENSOR_CRI));
        sleep_ms(200);
    }
     */
    /*
    sleep_ms(2000);

    while (true)
    {
        for (int i = 0; i < 2; i++)
        {
            _pickup(PICKUP_RIGHT_OUT_CURVE);
            _pickup(PICKUP_LEFT_OUT_CURVE);
        }
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

    if (_enable_unload)
        _motor_manager->spinUnload();
}

void Controller::_driveClosedLoop(double dt)
{
    int32_t position_error = _sensor_manager->getHorizontalPosition(SENSOR_ROW_FRONT);

#ifdef ENABLE_SLOW_MODE
    if (abs(position_error) > pid_drive_slow.offset && _getTimestamp() > _last_slow_timestamp + SLOW_MODE_DELAY)
    {
        _last_config = pid_drive_slow;
        _last_slow_timestamp = _getTimestamp();
    }
    else if (_getTimestamp() > _last_slow_timestamp + SLOW_MODE_DELAY)
        _last_config = _drive_config;
#else
    _last_config = _drive_config;
#endif

    double Pout = _last_config.kp * position_error;

    double derivative = (position_error - _last_error_drive) / dt;
    double Dout = _last_config.kd * derivative;

    double output = Pout + Dout;

    _last_error_drive = position_error;

    int32_t motor1_out = _last_config.speed + (int32_t)output / 4;
    int32_t motor2_out = _last_config.speed - (int32_t)output / 4;

    _motor_manager->drive_motor_right->setSpeed(motor1_out);
    _motor_manager->drive_motor_left->setSpeed(motor2_out);
}

/*
void Controller::_driveClosedLoop(double dt)
{
    int32_t sensor1_val = _sensor_manager->readSensor(SENSOR_FLI);
    int32_t sensor2_val = _sensor_manager->readSensor(SENSOR_FRI);

    if (sensor1_val < BLACK && sensor2_val < BLACK)
        sensor1_val = 500;

    int32_t position_error = 500 - sensor1_val;

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
 */

void Controller::_alignTangentialPID()
{
    _motor_manager->setSpeed(0);

    _integral_tan = 0;
    _last_error_tan = 0;

    int32_t position_error = _sensor_manager->getHorizontalPosition(SENSOR_ROW_BACK) - _sensor_manager->getHorizontalPosition(SENSOR_ROW_FRONT) - ALIGN_OFFSET;

    while(abs(position_error) > _align_config.offset)
    {
        position_error = _sensor_manager->getHorizontalPosition(SENSOR_ROW_BACK) - _sensor_manager->getHorizontalPosition(SENSOR_ROW_FRONT) - ALIGN_OFFSET;
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

    _error_stack.clear();

    int32_t position_error;

    while(abs(_error_stack.get()) > _align_config.offset || !_error_stack.isFull())
    {
        position_error = _sensor_manager->getFullHorizontalPosition(SENSOR_ROW_BACK) - _sensor_manager->getFullHorizontalPosition(SENSOR_ROW_FRONT);
        _error_stack.add(position_error);
        __spinAlignTangential(1000.0 / ALIGN_TAN_FREQ, position_error);
        sleep_ms(1000 / ALIGN_TAN_FREQ);
    }

    /*
    int32_t position_error = _sensor_manager->getFullHorizontalPosition(SENSOR_ROW_BACK) - _sensor_manager->getFullHorizontalPosition(SENSOR_ROW_FRONT);

    while(abs(position_error) > _align_config.offset)
    {
        position_error = _sensor_manager->getFullHorizontalPosition(SENSOR_ROW_BACK) - _sensor_manager->getFullHorizontalPosition(SENSOR_ROW_FRONT);
        __spinAlignTangential(1000.0 / ALIGN_TAN_FREQ, position_error);
        sleep_ms(1000 / ALIGN_TAN_FREQ);
    }
    */
    _motor_manager->setDirection(STOP);
}

void Controller::__spinAlignTangential(double dt, int32_t position_error)
{
    double Pout = _align_config.kp * position_error;

    _integral_tan += int32_t(dt * position_error);
    double Iout = _align_config.ki * _integral_tan;

    double derivative = (position_error - _last_error_tan) / dt;
    double Dout = _align_config.kd * derivative;

    double output = Pout + Dout + Iout;

    _last_error_tan = position_error;

    int32_t motor_out = (int32_t)output / 4;

    if (motor_out > 0)
    {
        _motor_manager->setSpeed(motor_out);
        _motor_manager->drive_motor_right->setDirection(FORWARD);
        _motor_manager->drive_motor_left->setDirection(BACKWARD);
    } else
    {
        _motor_manager->setSpeed(- motor_out);
        _motor_manager->drive_motor_right->setDirection(BACKWARD);
        _motor_manager->drive_motor_left->setDirection(FORWARD);
    }

    printf("pos_err: %li, Pout: %.2f, Dout %.2f, out %.2f\n\r", position_error, Pout, Dout, output);
}

void Controller::_alignRightHorizontal()
{
    for (int i = 0; i < 3; i++)
    {
        int32_t sensor_left = _sensor_manager->readSensor(SENSOR_CLO);

        _motor_manager->setSpeed(ALIGN_HOR_BASE_SPEED);
        if (sensor_left < ON_LINE)
            _motor_manager->setDirection(FORWARD);
        else
            _motor_manager->setDirection(BACKWARD);

        while (abs(_sensor_manager->readSensor(SENSOR_CLO) - ON_LINE) > OFFSET_ERROR_HORIZONTAL)
            sleep_ms(1);
        _motor_manager->setDirection(STOP);
        printf("Align straight");

        int32_t sensor_right = _sensor_manager->readSensor(SENSOR_CRO);

        _motor_manager->drive_motor_right->setSpeed(ALIGN_HOR_BASE_SPEED);
        if (sensor_right < ON_LINE)
            _motor_manager->drive_motor_right->setDirection(FORWARD);
        else
            _motor_manager->drive_motor_right->setDirection(BACKWARD);

        while (abs(_sensor_manager->readSensor(SENSOR_CRO) - ON_LINE) > OFFSET_ERROR_HORIZONTAL)
            sleep_ms(1);
        _motor_manager->setDirection(STOP);
        printf("Align right");
    }
}

void Controller::_alignLeftHorizontal()
{
    for (int i = 0; i < 3; i++)
    {
        int32_t sensor_right = _sensor_manager->readSensor(SENSOR_CRO);

        _motor_manager->setSpeed(ALIGN_HOR_BASE_SPEED);
        if (sensor_right < ON_LINE)
            _motor_manager->setDirection(FORWARD);
        else
            _motor_manager->setDirection(BACKWARD);

        while (abs(_sensor_manager->readSensor(SENSOR_CRO) - ON_LINE) > OFFSET_ERROR_HORIZONTAL)
            sleep_ms(1);
        _motor_manager->setDirection(STOP);
        printf("Align straight");

        int32_t sensor_left = _sensor_manager->readSensor(SENSOR_CLO);

        _motor_manager->drive_motor_left->setSpeed(ALIGN_HOR_BASE_SPEED);
        if (sensor_left < ON_LINE)
            _motor_manager->drive_motor_left->setDirection(FORWARD);
        else
            _motor_manager->drive_motor_left->setDirection(BACKWARD);

        while (abs(_sensor_manager->readSensor(SENSOR_CLO) - ON_LINE) > OFFSET_ERROR_HORIZONTAL)
            sleep_ms(1);
        _motor_manager->setDirection(STOP);
        printf("Align left");
    }
}

void Controller::_detectLine()
{
    /*
    if (_line_sensor == SENSOR_CLO)
    {
        if (_sensor_manager->readSensor(SENSOR_CLO) < BLACK && _sensor_manager->readSensor(SENSOR_CRI) < BLACK)
        {
            _motor_manager->setDirection(STOP);
            _enable_drive = false;
            _enable_detection = false;

            _executor->notify(NOTIFY_LINE);
        }
    } else
    {
        if (_sensor_manager->readSensor(SENSOR_CRO) < BLACK && _sensor_manager->readSensor(SENSOR_CLI) < BLACK)
        {
            _motor_manager->setDirection(STOP);
            _enable_drive = false;
            _enable_detection = false;

            _executor->notify(NOTIFY_LINE);
        }
    }
    */
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
    _alignRightHorizontal();

    _motor_manager->driveToUnload();

    for (int n = 0; n<_unload_counter/10; n++)
    {
        _motor_manager->spinUnload();
    }

    _motor_manager->driveFromUnload();
}

uint32_t Controller::_getTimestamp()
{
    auto time = get_absolute_time();
    return to_ms_since_boot(time);
}