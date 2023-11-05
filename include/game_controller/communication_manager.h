/**
 * @file communication_manager.h
 * @author Noa Sendlhofer
 */

#ifndef PICO_MOTORS_COMMUNICATION_MANAGER_H_
#define PICO_MOTORS_COMMUNICATION_MANAGER_H_

#include "pico/multicore.h"

enum GameMessage
{
    NOTIFY_READY,
    NOTIFY_CALIBRATE,
    NOTIFY_LINE,
    NOTIFY_TRASH,
    NOTIFY_UNLOAD,
    REQUEST_CALIBRATE,
    REQUEST_DRIVE_STRAIGHT,
    REQUEST_DRIVE_CURVE,
    REQUEST_LINE_LEFT,
    REQUEST_LINE_RIGHT,
    REQUEST_TRASH_LEFT,
    REQUEST_TRASH_RIGHT,
    REQUEST_UNLOAD,
};

class CommunicationManager
{
public:
    static bool sendMessage(GameMessage message);
    static bool checkInbox(GameMessage *message);
};


#endif //PICO_MOTORS_COMMUNICATION_MANAGER_H_
