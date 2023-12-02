/**
 * @file game_executor.cpp
 * @author Noa Sendlhofer
 */

#include <cstdio>
#include <stdexcept>
#include "game_executor.h"

void GameExecutor::notify(GameMessage status) const
{
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
            _controller->calibrate();
            break;
        case REQUEST_DRIVE_STRAIGHT:
            _controller->drive(BACKWARD, SPEED_STRAIGHT);
            break;
        case REQUEST_DRIVE_CURVE:
            _controller->drive(BACKWARD, SPEED_CURVE);
            break;
        case REQUEST_DRIVE_GATE:
            _controller->drive(BACKWARD, SPEED_GATE);
            break;
        case REQUEST_LINE_LEFT:
            _controller->detectLine(DETECT_LEFT);
            break;
        case REQUEST_LINE_RIGHT:
            _controller->detectLine(DETECT_RIGHT);
            break;
        case REQUEST_LINE_UNLOAD:
            _controller->detectLine(DETECT_UNLOAD);
            break;
        case REQUEST_ALIGN_STRAIGHT:
            _controller->align(STRAIGHT);
            break;
        case REQUEST_ALIGN_CURVE_LEFT:
            _controller->align(CURVE_LEFT);
            break;
        case REQUEST_ALIGN_CURVE_RIGHT:
            _controller->align(CURVE_RIGHT);
            break;
        case REQUEST_RESUME_CURVE_LEFT:
            _controller->resumeDrive(LEFT);
            break;
        case REQUEST_RESUME_CURVE_RIGHT:
            _controller->resumeDrive(RIGHT);
            break;
        case REQUEST_PICKUP_LEFT:
            _controller->pickTrash(PICKUP_LEFT);
            break;
        case REQUEST_PICKUP_LEFT_SHIELD:
            _controller->pickTrash(PICKUP_LEFT_SHIELD);
            break;
        case REQUEST_PICKUP_LEFT_CURVE:
            _controller->pickTrash(PICKUP_LEFT_CURVE);
            break;
        case REQUEST_PICKUP_RIGHT:
            _controller->pickTrash(PICKUP_RIGHT);
            break;
        case REQUEST_PICKUP_RIGHT_SHIELD:
            _controller->pickTrash(PICKUP_RIGHT_SHIELD);
            break;
        case REQUEST_PICKUP_RIGHT_CURVE:
            _controller->pickTrash(PICKUP_RIGHT_CURVE);
            break;
        case REQUEST_UNLOAD:
            _controller->unload();
            break;
        case REQUEST_MIXER_ON:
            _controller->setMixer(true);
            break;
        case REQUEST_MIXER_OFF:
            _controller->setMixer(false);
            break;
        case REQUEST_WIGGLE:
            _controller->wiggle();
            break;
        default:
            throw std::runtime_error("Invalid message");
    }
}
