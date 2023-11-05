/**
 * @file game_controller.cpp
 * @author Noa Sendlhofer
 */

#include <stdexcept>
#include "game_controller.h"

void GameController::checkInbox()
{
    GameMessage message;
    if (!CommunicationManager::checkInbox(&message))
        return;

    switch (message)
    {
        case NOTIFY_READY:
            _controller_ready = true;
            _nextMove(message);
            break;
        case NOTIFY_LINE:
            _nextMove(message);
            break;
        case NOTIFY_CALIBRATE:
            _nextMove(message);
            break;
        case NOTIFY_TRASH:
            _nextMove(message);
            break;
        case NOTIFY_UNLOAD:
            _nextMove(message);
            break;
        default:
            throw(std::runtime_error("Invalid message"));
    }
}

void GameController::_nextMove(GameMessage message)
{
    if (!_controller_ready)
        throw(std::runtime_error("Controller offline"));

    auto time_passed = _getTimestamp() - _last_action_timestamp;

    switch (message)
    {
        case NOTIFY_READY:
            break;
        case NOTIFY_CALIBRATE:
            if (time_passed < CALIBRATION_TIME)
                throw(std::runtime_error("Calibration unsuccessful")); //TODO implement safe
            break;
        case NOTIFY_LINE:
            if (time_passed < LINE_TIME)
            {
                _game_index = _game_index - 2;
                _sendMove();
            }
            break;
        case NOTIFY_TRASH:
            if (time_passed < TRASH_TIME)
                throw(std::runtime_error("Trash pickup unsuccessful")); //TODO implement safe
            break;
        case NOTIFY_UNLOAD:
            if (time_passed < UNLOAD_TIME)
                throw(std::runtime_error("Unload unsuccessful")); //TODO implement safe
            break;
        default:
            throw(std::runtime_error("Invalid message"));
    }

    _sendMove();
}

void GameController::_sendMove()
{
    switch (game_plan.at(_game_index))
    {
        case CALIBRATE:
            CommunicationManager::sendMessage(REQUEST_CALIBRATE);
            break;
        case DRIVE_STRAIGHT:
            CommunicationManager::sendMessage(REQUEST_DRIVE_STRAIGHT);
            if (game_plan.at(_game_index + 1) == LINE_RIGHT || game_plan.at(_game_index + 1) == LINE_LEFT)
            {
                _last_action_timestamp = _getTimestamp();
                sleep_ms(LINE_WAIT_TIME);
                _game_index++;
                _sendMove();
                return;
            }
            break;
        case DRIVE_CURVE:
            CommunicationManager::sendMessage(REQUEST_DRIVE_CURVE);
            if (game_plan.at(_game_index + 1) == LINE_RIGHT || game_plan.at(_game_index + 1) == LINE_LEFT)
            {
                _last_action_timestamp = _getTimestamp();
                sleep_ms(LINE_WAIT_TIME);
                _game_index++;
                _sendMove();
                return;
            }
            break;
        case DRIVE_GATE:
            //TODO
            break;
        case LINE_LEFT:
            CommunicationManager::sendMessage(REQUEST_LINE_LEFT);
            break;
        case LINE_RIGHT:
            CommunicationManager::sendMessage(REQUEST_LINE_RIGHT);
            break;
        case TRASH_LEFT:
            CommunicationManager::sendMessage(REQUEST_TRASH_LEFT);
            break;
        case TRASH_RIGHT:
            CommunicationManager::sendMessage(REQUEST_TRASH_RIGHT);
            break;
        case UNLOAD:
            CommunicationManager::sendMessage(REQUEST_UNLOAD);
            break;
    }

    _last_action_timestamp = _getTimestamp();
    _game_index++;
}

uint32_t GameController::_getTimestamp()
{
    auto time = get_absolute_time();
    return to_ms_since_boot(time);
}
