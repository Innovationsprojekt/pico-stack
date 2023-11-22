/**
 * @file controller.h
 * @author Noa Sendlhofer
 */

#ifndef PICO_MOTORS_CONTROLLER_H_
#define PICO_MOTORS_CONTROLLER_H_

#include <memory>
#include "motor_manager.h"
#include "sensor_manager.h"
#include "controller_interface.h"

#include "enum_definitions.h"

//#define ENABLE_SENSOR_CALIB
#define ENABLE_PICKUP
#define ENABLE_UNLOAD
#define ENABLE_MIXER

#define OFFSET_ERROR_TANGENTIAL 70
#define OFFSET_ERROR_HORIZONTAL 70

#define ALIGN_TAN_BASE_SPEED 1000
#define ALIGN_HOR_BASE_SPEED 1700

#define ALIGN_TAN_KD 7
#define ALIGN_TAN_KP 8
#define ALIGN_TAN_KI 1
#define ALIGN_TAN_FREQ 500

#define DRIVE_STRAIGHT_KD 70
#define DRIVE_STRAIGHT_KP 85
#define DRIVE_STRAIGHT_SPEED 7000

#define DRIVE_CURVE_KD 35
#define DRIVE_CURVE_KP 55
#define DRIVE_CURVE_SPEED 6000

#define MIXER_SPEED 1800

class Controller : public ControllerInterface
{
public:
    Controller();
    void start();
    void spin(double dt);

    void calibrate() const override;
    void drive(MotorDirection direction, DriveSpeed speed) override;
    void detectLine(LineSide side) override;
    void align(LineType type) override;
    void pickTrash(PickUpSide side) const override;
    void resumeDrive(TurnDirection dir) override;
    void unload() const override;
    void setMixer(bool enabled) override;

private:
    std::shared_ptr<MotorManager> _motor_manager;
    std::shared_ptr<SensorManager> _sensor_manager;

    void _alignHorizontal();

    void _alignTangentialPID();
    void __spinAlignTangential(double dt);

    // drive
    void _driveClosedLoop(double dt);

    bool _enable_drive = false;
    uint16_t _drive_speed = DRIVE_STRAIGHT_SPEED;
    uint8_t _drive_kd = DRIVE_STRAIGHT_KD;
    uint8_t _drive_kp = DRIVE_STRAIGHT_KP;
    int32_t _last_error_drive = 0;

    // detect line
    void _detectLine();
    bool _enable_detection = false;
    SensorPosition _line_sensor = SENSOR_CLO;

    // align
    int32_t _last_error_tan = 0;
    int32_t _integral_tan = 0;

    // pick trash
    void _pickup(PickUpSide side) const;

    // unload
    void _unload() const;
};


#endif //PICO_MOTORS_CONTROLLER_H_
