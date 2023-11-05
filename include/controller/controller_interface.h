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

    virtual void drive() const = 0;
    virtual void pickTrash() const = 0; //TODO add all methods

protected:
    GameExecutorInterface *_executor;
};


#endif //PICO_MOTORS_CONTROLLER_INTERFACE_H_
