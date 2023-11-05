/**
 * @file game_executor.cpp
 * @author Noa Sendlhofer
 */

#include <cstdio>
#include <stdexcept>
#include "game_executor.h"

void GameExecutor::notify(GameMessage status) const
{
    printf("Controller has notified!");

    CommunicationManager::sendMessage(status);
}

void GameExecutor::checkInbox()
{
    GameMessage message;
    if (!CommunicationManager::checkInbox(&message))
        return;

    switch (message)
    {
        case REQUEST_CALIBRATE:
            //TODO
            break;
        case REQUEST_DRIVE_STRAIGHT:
            _controller->drive();
            break;
        case REQUEST_PICKUP:
            _controller->pickTrash();
            break;
        case REQUEST_UNLOAD:
            //TODO
            break;
        default:
            throw(std::runtime_error("Invalid Message"));
    }
}
