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

#define OFFSET_ERROR_TANGENTIAL_STRAIGHT 120
#define OFFSET_ERROR_TANGENTIAL_CURVE 100
#define OFFSET_ERROR_HORIZONTAL 70

#define ALIGN_TAN_BASE_SPEED 1000
#define ALIGN_HOR_BASE_SPEED 2500

#define ALIGN_TAN_FREQ 500

#define ALIGN_ST_TAN_KD 10
#define ALIGN_ST_TAN_KP 2
#define ALIGN_ST_TAN_KI 1.2
/*
#define ALIGN_CUV_TAN_KD 10
#define ALIGN_CUV_TAN_KP 1.2
#define ALIGN_CUV_TAN_KI 1.2
 */

#define ALIGN_CUV_TAN_KD 9
#define ALIGN_CUV_TAN_KP 0.2
#define ALIGN_CUV_TAN_KI 0.5

#define DRIVE_STRAIGHT_KD 57
#define DRIVE_STRAIGHT_KP 90
#define DRIVE_STRAIGHT_SPEED 8000

#define DRIVE_CURVE_KD 35
#define DRIVE_CURVE_KP 75
#define DRIVE_CURVE_SPEED 7000

#define DRIVE_GATE_KD 55
#define DRIVE_GATE_KP 85
#define DRIVE_GATE_SPEED 7000

#define MIXER_SPEED 4500

#define UNLOAD_CLOSE 15
#define UNLOAD_OPEN 180

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
    void unload() override;
    void setMixer(bool enabled) override;
    void wiggle() override;

private:
    std::shared_ptr<MotorManager> _motor_manager;
    std::shared_ptr<SensorManager> _sensor_manager;

    void _alignHorizontal();

    void _alignTangentialPID();
    void _alignFullTangentialPID();
    void __spinAlignTangential(double dt, int32_t position_error);

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
    double _align_kp = ALIGN_ST_TAN_KP;
    double _align_kd = ALIGN_ST_TAN_KD;
    double _align_ki = ALIGN_ST_TAN_KI;
    uint16_t _offset = OFFSET_ERROR_TANGENTIAL_STRAIGHT;

    // pick trash
    void _pickup(PickUpSide side) const;

    // unload
    void _unload();
    uint16_t _unload_counter = 30;
};


#endif //PICO_MOTORS_CONTROLLER_H_
