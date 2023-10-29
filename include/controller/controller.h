/**
 * @file controller.h
 * @author Noa Sendlhofer
 */

#ifndef PICO_MOTORS_CONTROLLER_H_
#define PICO_MOTORS_CONTROLLER_H_


#include <memory>
#include "motor_manager.h"
#include "sensor_manager.h"

#define OFFSET_ERROR_TANGENTIAL 50
#define OFFSET_ERROR_HORIZONTAL 50

enum GamePosition
{
    STRAIGHT,
    TURN_RIGHT,
    TURN_LEFT,
};

class Controller
{
public:
    Controller(std::shared_ptr<MotorManager> motor_manager, std::shared_ptr<SensorManager> sensor_manager);
    void spin(double dt);

private:
    std::shared_ptr<MotorManager> motor_manager;
    std::shared_ptr<SensorManager> sensor_manager;

    void _driveClosedLoop(double dt);
    void _checkGamePosition();

    int32_t __readLineSensor(GamePosition current_pos);
    Direction __findTurnDirection();

    void _alignTangential(Direction turn_direction);
    void _alignHorizontal();

    int32_t _last_error = 0;
    const int16_t _Kd = 15;
    const int16_t _Kp = 30;
    const uint32_t _base_speed = 1000;

    const std::vector<GamePosition> __game_configuration = {STRAIGHT, TURN_LEFT, TURN_LEFT, STRAIGHT};
    uint16_t _current_position = 0;
};


#endif //PICO_MOTORS_CONTROLLER_H_
