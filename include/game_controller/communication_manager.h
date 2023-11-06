/**
 * @file communication_manager.h
 * @author Noa Sendlhofer
 */

#ifndef PICO_MOTORS_COMMUNICATION_MANAGER_H_
#define PICO_MOTORS_COMMUNICATION_MANAGER_H_

#include "pico/multicore.h"
#include "enum_definitions.h"

class CommunicationManager
{
public:
    static bool sendMessage(GameMessage message);
    static bool checkInbox(GameMessage *message);
};


#endif //PICO_MOTORS_COMMUNICATION_MANAGER_H_
