/**
 * @file game_controller.h
 * @author Noa Sendlhofer
 */

#ifndef PICO_MOTORS_GAME_CONTROLLER_H_
#define PICO_MOTORS_GAME_CONTROLLER_H_

#include <vector>
#include "communication_manager.h"
#include "enum_definitions.h"

#define LINE_WAIT_TIME 1000

#define CALIBRATION_TIME 2000
#define LINE_TIME 1000
#define PICKUP_TIME 4000
#define UNLOAD_TIME 10000

const std::vector<GameItems> game_plan
    = {CALIBRATE,
       DRIVE_STRAIGHT,
       LINE_RIGHT,
       ALIGN_STRAIGHT,
       TRASH_RIGHT,
       DRIVE_STRAIGHT,
       LINE_LEFT,
       ALIGN_STRAIGHT,
       TRASH_LEFT,
       DRIVE_CURVE,
       LINE_RIGHT,
       ALIGN_CURVE_LEFT,
       TRASH_RIGHT,
       RESUME_CURVE_LEFT,
       DRIVE_CURVE,
       LINE_RIGHT,
       ALIGN_CURVE_LEFT,
       TRASH_LEFT,
       RESUME_CURVE_LEFT,
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
