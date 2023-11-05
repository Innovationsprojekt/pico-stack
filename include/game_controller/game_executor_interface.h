/**
 * @file game_executor_interface.h
 * @author Noa Sendlhofer
 */

#ifndef PICO_MOTORS_GAME_EXECUTOR_INTERFACE_H_
#define PICO_MOTORS_GAME_EXECUTOR_INTERFACE_H_

#include "communication_manager.h"

class GameExecutorInterface
{
public:
    virtual void notify(GameMessage status) const = 0;
};


#endif //PICO_MOTORS_GAME_EXECUTOR_INTERFACE_H_
