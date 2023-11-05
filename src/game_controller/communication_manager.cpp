/**
 * @file communication_manager.cpp
 * @author Noa Sendlhofer
 */

#include "communication_manager.h"

bool CommunicationManager::sendMessage(GameMessage message)
{
    if (multicore_fifo_wready())
    {
        multicore_fifo_push_blocking(message);
        return true;
    }

    return false;
}

bool CommunicationManager::checkInbox(GameMessage *message)
{
    if (multicore_fifo_rvalid())
    {
        *message = static_cast<GameMessage>(multicore_fifo_pop_blocking());
        return true;
    }

    return false;
}
