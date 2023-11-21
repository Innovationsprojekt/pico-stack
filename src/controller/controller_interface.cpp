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
            _enable_drive = true;
            break;
        case SPEED_CURVE:
            _drive_kp = DRIVE_CURVE_KP;
            _drive_kd = DRIVE_CURVE_KD;
            _drive_speed = DRIVE_CURVE_SPEED;
            _enable_drive = true;
            break;
    }
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
    }
    _enable_detection = true;
}

void Controller::align(LineType type)
{
    switch (type)
    {
        case STRAIGHT:
            _alignTangentialPID();
            sleep_ms(200);
            _alignHorizontal();
            sleep_ms(200);
            _alignTangentialPID();
            break;
        case CURVE_LEFT:
            _motor_manager->turn(10, RIGHT);

            _alignTangentialPID();
            sleep_ms(200);
            _alignHorizontal();
            sleep_ms(200);
            _alignTangentialPID();
            break;
        case CURVE_RIGHT:
            _motor_manager->turn(10, LEFT);

            _alignTangentialPID();
            sleep_ms(200);
            _alignHorizontal();
            sleep_ms(200);
            _alignTangentialPID();
            break;
    }

    _executor->notify(NOTIFY_ALIGN);
}

void Controller::pickTrash(PickUpSide side) const
{
    if (ENABLE_PICKUP)
        _pickup(side);

    _executor->notify(NOTIFY_TRASH);
}

void Controller::unload() const
{
    if (ENABLE_UNLOAD)
        _unload();

    _executor->notify(NOTIFY_UNLOAD);
}

void Controller::resumeDrive(TurnDirection dir)
{
    switch (dir)
    {
        case LEFT:
            _motor_manager->creepDistance(2, FORWARD);
            _motor_manager->setDirection(STOP);
            _motor_manager->turn(15, RIGHT);
            _motor_manager->creepDistance(2, BACKWARD);
            break;
        case RIGHT:
            _motor_manager->creepDistance(2, FORWARD);
            _motor_manager->setDirection(STOP);
            _motor_manager->turn(15, LEFT);
            _motor_manager->creepDistance(2, BACKWARD);
            break;
    }

    _executor->notify(NOTIFY_RESUME);
}

void Controller::setMixer(bool enabled)
{
    if (!ENABLE_MIXER)
        return;

    if (enabled)
    {
        _motor_manager->setMixerSpeed(MIXER_SPEED);
        _motor_manager->setMixerDirection(FORWARD);
    }
    else
        _motor_manager->setMixerDirection(STOP);
}
