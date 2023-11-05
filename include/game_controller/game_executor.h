/**
 * @file game_executor.h
 * @author Noa Sendlhofer
 */

#ifndef PICO_MOTORS_GAME_EXECUTOR_H_
#define PICO_MOTORS_GAME_EXECUTOR_H_

#include "game_executor_interface.h"
#include "controller_interface.h"
#include "communication_manager.h"

#include "enum_definitions.h"

class GameExecutor : public GameExecutorInterface
{
public:
    explicit GameExecutor(ControllerInterface *controller)
        : _controller(controller)
    {
    }

    void checkInbox();
    void notify(GameMessage status) const override;

private:
    ControllerInterface *_controller;
};

#endif //PICO_MOTORS_GAME_EXECUTOR_H_