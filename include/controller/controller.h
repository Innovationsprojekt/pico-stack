/**
 * @file controller.h
 * @author Noa Sendlhofer
 */

#ifndef PICO_MOTORS_CONTROLLER_H_
#define PICO_MOTORS_CONTROLLER_H_


#include <memory>
#include "motor_manager.h"
#include "sensor_manager.h"

#define OFFSET_ERROR 20

class Controller
{
public:
    Controller(std::shared_ptr<MotorManager> motor_manager, std::shared_ptr<SensorManager> sensor_manager);
    void spin(double dt);
private:
    std::shared_ptr<MotorManager> motor_manager;
    std::shared_ptr<SensorManager> sensor_manager;

    void _driveClosedLoop(double dt);
    void _alignTangential();

    int32_t _last_error = 0;
    const int16_t _Kd = 10;
    const int16_t _Kp = 40;
    const uint32_t base_speed = 1500;
};


#endif //PICO_MOTORS_CONTROLLER_H_
