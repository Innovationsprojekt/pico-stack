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

#define OFFSET_ERROR_TANGENTIAL 50
#define OFFSET_ERROR_HORIZONTAL 50

#define ALIGN_TAN_BASE_SPEED 800
#define ALIGN_HOR_BASE_SPEED 1000

#define ALIGN_TAN_KD 5
#define ALIGN_TAN_KP 7
#define ALIGN_TAN_KI 1
#define ALIGN_TAN_FREQ 500

#define DRIVE_KD 25
#define DRIVE_KP 50
#define DRIVE_BASE_SPEED 6000

#define DRIVE_CURVE_KD 15
#define DRIVE_CURVE_KP 30
#define DRIVE_CURVE_SPEED 4000

enum GamePosition
{
    STRAIGHT,
    TURN_RIGHT,
    TURN_LEFT,
};

class Controller : public ControllerInterface
{
public:
    Controller(std::shared_ptr<MotorManager> motor_manager, std::shared_ptr<SensorManager> sensor_manager);
    void spin(double dt);

    void drive() const override;
    void pickTrash() const override;

private:
    std::shared_ptr<MotorManager> motor_manager;
    std::shared_ptr<SensorManager> sensor_manager;

    void _driveClosedLoop(double dt);
    void _checkGamePosition();

    int32_t __readLineSensor(GamePosition current_pos);
    Direction __findTurnDirection();

    void _alignTangential(Direction turn_direction);
    void _alignHorizontal();

    void _alignTangentialPID();
    void __spinAlignTangential(double dt);

    uint16_t _drive_speed = DRIVE_BASE_SPEED;
    uint8_t _drive_kd = DRIVE_KD;
    uint8_t _drive_kp = DRIVE_KP;
    int32_t _last_error_drive = 0;

    int32_t _last_error_tan = 0;
    int32_t _integral_tan = 0;

    const std::vector<GamePosition> __game_configuration = {STRAIGHT, TURN_LEFT, TURN_LEFT, STRAIGHT};
    uint16_t _current_position = 0;
};


#endif //PICO_MOTORS_CONTROLLER_H_
