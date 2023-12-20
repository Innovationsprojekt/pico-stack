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
            _drive_config = pid_drive_st;
            break;
        case SPEED_CURVE:
            _drive_config = pid_drive_cuv;
            break;
        case SPEED_GATE:
            _drive_config = pid_drive_gate;
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
            _align_config = pid_align_st_tan;

            _alignRightHorizontal();
            _alignLeftHorizontal();
            break;
        case CURVE_LEFT:
            _motor_manager->turn(10, LEFT);

            _align_config = pid_align_cuv_tan;

            _alignRightHorizontal();
            _alignRightHorizontal();
            break;
        case CURVE_RIGHT:
            _motor_manager->turn(10, RIGHT);

            _align_config = pid_align_cuv_tan;

            _alignLeftHorizontal();
            _alignLeftHorizontal();
            break;
    }

    _executor->notify(NOTIFY_ALIGN);
}

void Controller::pickTrash(PickUpSide side) const
{
#ifdef ENABLE_PICKUP
    _pickup(side);
#else
    sleep_ms(3000);
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

void Controller::unloadStay()
{
    _alignRightHorizontal();

    _motor_manager->driveToUnload();
    _enable_unload = true;
}

void Controller::goal()
{
    _enable_unload = false;
    _motor_manager->driveFromUnload();

    _motor_manager->setSpeed(10000);
    _motor_manager->setDirection(BACKWARD);
    sleep_ms(8000);
    _motor_manager->setDirection(STOP);

    _executor->notify(NOTIFY_UNLOAD);
}

void Controller::resumeDrive(ResumeDriveType dir)
{
    switch (dir)
    {
        case RESUME_IN_CURVE_LEFT:
            _motor_manager->turn(13, RIGHT);
            _motor_manager->creepDistance(8, BACKWARD);
            _motor_manager->drive_motor_right->setSpeed(10000);
            _motor_manager->drive_motor_left->setSpeed(2600);
            _motor_manager->drive_motor_right->setDirection(BACKWARD);
            _motor_manager->drive_motor_left->setDirection(BACKWARD);
            sleep_ms(2000);
            _motor_manager->setDirection(STOP);
            _motor_manager->turn(15, LEFT);
            _motor_manager->creepDistance(4, FORWARD);
            _motor_manager->turn(5, RIGHT);
            break;
        case RESUME_OUT_CURVE_LEFT:
            _motor_manager->turn(13, RIGHT);
            _motor_manager->creepDistance(8, BACKWARD);
            _motor_manager->drive_motor_right->setSpeed(10000);
            _motor_manager->drive_motor_left->setSpeed(2800);
            _motor_manager->drive_motor_right->setDirection(BACKWARD);
            _motor_manager->drive_motor_left->setDirection(BACKWARD);
            sleep_ms(1400);
            _motor_manager->setDirection(STOP);
            break;
        case RESUME_IN_CURVE_RIGHT:
            _motor_manager->turn(13, LEFT);
            _motor_manager->creepDistance(8, BACKWARD);
            _motor_manager->drive_motor_left->setSpeed(10000);
            _motor_manager->drive_motor_right->setSpeed(2600);
            _motor_manager->drive_motor_left->setDirection(BACKWARD);
            _motor_manager->drive_motor_right->setDirection(BACKWARD);
            sleep_ms(2000);
            _motor_manager->setDirection(STOP);
            _motor_manager->turn(15, RIGHT);
            _motor_manager->creepDistance(4, FORWARD);
            _motor_manager->turn(5, LEFT);
            break;
        case RESUME_OUT_CURVE_RIGHT:
            _motor_manager->turn(13, LEFT);
            _motor_manager->creepDistance(8, BACKWARD);
            _motor_manager->drive_motor_left->setSpeed(10000);
            _motor_manager->drive_motor_right->setSpeed(2800);
            _motor_manager->drive_motor_left->setDirection(BACKWARD);
            _motor_manager->drive_motor_right->setDirection(BACKWARD);
            sleep_ms(1400);
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
    }
#endif
}

void Controller::wiggle()
{
    _motor_manager->setDirection(STOP);
    _enable_drive = false;

    _motor_manager->setMixerDirection(FORWARD);
    sleep_ms(1000);
    _motor_manager->setMixerDirection(BACKWARD);

    _motor_manager->setSpeed(10000);
    for (int i = 0; i<5; i++)
    {
        //_motor_manager->setMixerDirection(FORWARD);
        _motor_manager->setDirection(FORWARD);
        sleep_ms(150);
        //_motor_manager->setMixerDirection(BACKWARD);
        _motor_manager->setDirection(BACKWARD);
        sleep_ms(150);
    }
    _motor_manager->setDirection(STOP);

    _executor->notify(NOTIFY_WIGGLE);
}