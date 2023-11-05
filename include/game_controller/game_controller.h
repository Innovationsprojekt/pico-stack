/**
 * @file game_controller.h
 * @author Noa Sendlhofer
 */

#ifndef PICO_MOTORS_GAME_CONTROLLER_H_
#define PICO_MOTORS_GAME_CONTROLLER_H_

#include <vector>
#include "communication_manager.h"

#define LINE_WAIT_TIME 1000

#define CALIBRATION_TIME 2000
#define LINE_TIME 2000
#define TRASH_TIME 6000
#define UNLOAD_TIME 10000

enum GameItems
{
    CALIBRATE,
    DRIVE_STRAIGHT,
    DRIVE_CURVE,
    DRIVE_GATE,
    LINE_LEFT,
    LINE_RIGHT,
    TRASH_LEFT,
    TRASH_RIGHT,
    UNLOAD,
};

const std::vector<GameItems> game_plan
        = {CALIBRATE,
           DRIVE_STRAIGHT,
           LINE_RIGHT,
           TRASH_RIGHT,
           DRIVE_STRAIGHT,
           LINE_LEFT,
           TRASH_LEFT,
           DRIVE_CURVE,
           LINE_RIGHT,
           TRASH_RIGHT,
           DRIVE_CURVE,
           LINE_RIGHT,
           TRASH_LEFT,
           DRIVE_STRAIGHT,
           UNLOAD
        };

class GameController
{
public:
    void checkInbox();
private:
    void _nextMove(GameMessage message);
    void _sendMove();

    static uint32_t _getTimestamp();

    uint16_t _game_index = 0;
    bool _controller_ready = false;

    uint32_t _last_action_timestamp = _getTimestamp();
};


#endif //PICO_MOTORS_GAME_CONTROLLER_H_
