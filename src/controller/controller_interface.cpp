/**
 * @file controller_interface.cpp
 * @author Noa Sendlhofer
 */

#include "controller.h"

void Controller::calibrate() const
{
    _sensor_manager->calibrate();
    _executor->notify(NOTIFY_CALIBRATE);
}

void Controller::drive(MotorDirection direction, DriveSpeed speed)
{
    _motor_manager->setDirection(direction);

    switch (speed)
    {
        case SPEED_STRAIGHT:
            _drive_kp = DRIVE_STRAIGHT_KP;
            _drive_kd = DRIVE_STRAIGHT_KD;
            _drive_speed = DRIVE_STRAIGHT_SPEED;
            break;
        case SPEED_CURVE:
            _drive_kp = DRIVE_CURVE_KP;
            _drive_kd = DRIVE_CURVE_KD;
            _drive_speed = DRIVE_CURVE_SPEED;
            break;
        case SPEED_GATE:
            _drive_kp = DRIVE_GATE_KP;
            _drive_kd = DRIVE_GATE_KD;
            _drive_speed = DRIVE_GATE_SPEED;
            break;
    }

    _enable_drive = true;
}

void Controller::detectLine(LineSide side)
{
    switch (side)
    {
        case DETECT_LEFT:
            _line_sensor = SENSOR_CLO;
            break;
        case DETECT_RIGHT:
            _line_sensor = SENSOR_CRO;
            break;
        case DETECT_UNLOAD:
            _line_sensor = SENSOR_FRO;
            break;
    }
    _enable_detection = true;
}

void Controller::align(LineType type)
{
    switch (type)
    {
        case STRAIGHT:
            _align_kd = ALIGN_ST_TAN_KD;
            _align_kp = ALIGN_ST_TAN_KP;
            _align_ki = ALIGN_ST_TAN_KI;
            _offset = OFFSET_ERROR_TANGENTIAL_STRAIGHT;

            _alignTangentialPID();
            _alignHorizontal();
            break;
        case CURVE_LEFT:
            _motor_manager->turn(10, LEFT);

            _align_kd = ALIGN_CUV_TAN_KD;
            _align_kp = ALIGN_CUV_TAN_KP;
            _align_ki = ALIGN_CUV_TAN_KI;
            _offset = OFFSET_ERROR_TANGENTIAL_CURVE;

            _alignHorizontal();
            _alignFullTangentialPID();
            _alignHorizontal();
            _alignFullTangentialPID();
            break;
        case CURVE_RIGHT:
            _motor_manager->turn(10, RIGHT);

            _align_kd = ALIGN_CUV_TAN_KD;
            _align_kp = ALIGN_CUV_TAN_KP;
            _align_ki = ALIGN_CUV_TAN_KI;
            _offset = OFFSET_ERROR_TANGENTIAL_CURVE;

            _alignHorizontal();
            _alignFullTangentialPID();
            _alignHorizontal();
            _alignFullTangentialPID();
            break;
    }

    _executor->notify(NOTIFY_ALIGN);
}

void Controller::pickTrash(PickUpSide side) const
{
#ifdef ENABLE_PICKUP
    _pickup(side);
#endif

    _executor->notify(NOTIFY_TRASH);
}

void Controller::unload()
{
#ifdef ENABLE_UNLOAD
    _unload();
#endif

    _executor->notify(NOTIFY_UNLOAD);
}

void Controller::resumeDrive(TurnDirection dir)
{
    switch (dir)
    {
        case LEFT:
            _motor_manager->turn(15, RIGHT);
            _motor_manager->creepDistance(8, BACKWARD);
            _motor_manager->drive_motor1->setSpeed(7000);
            _motor_manager->drive_motor2->setSpeed(1300);
            _motor_manager->drive_motor1->setDirection(BACKWARD);
            _motor_manager->drive_motor2->setDirection(BACKWARD);
            sleep_ms(1800);
            _motor_manager->setDirection(STOP);
            break;
        case RIGHT:
            _motor_manager->turn(15, LEFT);
            _motor_manager->creepDistance(8, BACKWARD);
            _motor_manager->drive_motor2->setSpeed(7000);
            _motor_manager->drive_motor1->setSpeed(1300);
            _motor_manager->drive_motor2->setDirection(BACKWARD);
            _motor_manager->drive_motor1->setDirection(BACKWARD);
            sleep_ms(1800);
            _motor_manager->setDirection(STOP);
            break;
    }

    _executor->notify(NOTIFY_RESUME);
}

void Controller::setMixer(bool enabled)
{
#ifdef ENABLE_MIXER
    if (enabled)
    {
        _motor_manager->setMixerSpeed(MIXER_SPEED);
        _motor_manager->setMixerDirection(BACKWARD);
    }
    else
    {
        _motor_manager->setMixerDirection(STOP);
        _motor_manager->setSpeed(10000);
        _motor_manager->setDirection(BACKWARD);
        sleep_ms(8000);
        _motor_manager->setDirection(STOP);
    }
#endif
}

void Controller::wiggle()
{
    _motor_manager->setDirection(STOP);
    _enable_drive = false;

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

    _executor->notify(NOTIFY_WIGGLE);
}
