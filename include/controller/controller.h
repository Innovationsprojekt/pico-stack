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
#include "error_stack.h"

// ----------- PARAMETERS -----------
#define MIXER_SPEED 4500

#define UNLOAD_WIGGLES 20

// ----------- ALIGN HORIZONTAL -----------
#define OFFSET_ERROR_HORIZONTAL 70
#define ALIGN_HOR_BASE_SPEED 2500

// ----------- ALIGN TANGENTIAL -----------
#define ALIGN_TAN_FREQ 500

const PIDConfiguration pid_align_st_tan = {1.5, 15, 1, 0, 120};
/* only inner sensors
 * const PIDConfiguration pid_align_st_tan = {2, 10, 1.2, 0};
 */

const PIDConfiguration pid_align_cuv_tan = {0.3, 15, 0.6, 0, 400};
/* only inner sensors
 * const PIDConfiguration pid_align_cuv_tan = {1.2, 10, 1.2, 0};
 */

// ----------- DRIVE PID -----------
#define ENABLE_SLOW_MODE

const PIDConfiguration pid_drive_st = {90, 57, 0, 8000, 0};
/* only 1 sensor
 * const PIDConfiguration pid_drive_st = {100, 55, 0, 8000, 0};
 */

const PIDConfiguration pid_drive_slow = {60, 30, 0, 4000, 500};

const PIDConfiguration pid_drive_cuv = {75, 37, 0, 7000, 0};

const PIDConfiguration pid_drive_gate = {85, 55, 0, 7000, 0};


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
    void unloadStay() override;
    void goal() override;
    void setMixer(bool enabled) override;
    void wiggle() override;

private:
    std::shared_ptr<MotorManager> _motor_manager;
    std::shared_ptr<SensorManager> _sensor_manager;

    // align horizontal
    void _alignHorizontal();

    // align tangential
    void _alignTangentialPID();
    void _alignFullTangentialPID();
    void __spinAlignTangential(double dt, int32_t position_error);
    int32_t _last_error_tan = 0;
    int32_t _integral_tan = 0;
    PIDConfiguration _align_config = pid_align_st_tan;

    // drive
    void _driveClosedLoop(double dt);
    bool _enable_drive = false;
    PIDConfiguration _drive_config = pid_drive_st;
    PIDConfiguration _last_config = pid_drive_st;
    int32_t _last_error_drive = 0;
    ErrorStack _error_stack;

    // detect line
    void _detectLine();
    bool _enable_detection = false;
    SensorPosition _line_sensor = SENSOR_CLO;

    // pick trash
    void _pickup(PickUpSide side) const;

    // unload
    void _unload();
    bool _enable_unload = false;
    uint16_t _unload_counter = UNLOAD_WIGGLES;
};


#endif //PICO_MOTORS_CONTROLLER_H_
