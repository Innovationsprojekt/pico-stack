/**
 * @file controller_interface.h
 * @author Noa Sendlhofer
 */

#ifndef PICO_MOTORS_CONTROLLER_INTERFACE_H_
#define PICO_MOTORS_CONTROLLER_INTERFACE_H_

#include <memory>
#include "game_executor_interface.h"

class ControllerInterface
{
public:
    void setExecutor(GameExecutorInterface *executor)
    {
        _executor = executor;
    }

    virtual void calibrate() const = 0;
    virtual void drive(MotorDirection direction, DriveSpeed speed) = 0;
    virtual void detectLine(LineSide side) = 0;
    virtual void align(LineType type) = 0;
    virtual void pickTrash(PickUpSide side) const = 0;
    virtual void resumeDrive(TurnDirection dir) = 0;
    virtual void unload() const = 0;

protected:
    GameExecutorInterface *_executor;
};


#endif //PICO_MOTORS_CONTROLLER_INTERFACE_H_
