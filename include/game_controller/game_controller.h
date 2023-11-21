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
       MIXER_ON,
       TRASH_RIGHT,

       DRIVE_STRAIGHT,
       LINE_LEFT,
       ALIGN_STRAIGHT,
       TRASH_LEFT,

       DRIVE_CURVE,
       LINE_LEFT,
       ALIGN_CURVE_LEFT,
       TRASH_RIGHT,

       DRIVE_CURVE,
       LINE_LEFT,
       ALIGN_CURVE_LEFT,
       TRASH_LEFT,

       DRIVE_STRAIGHT,
       LINE_LEFT,
       MIXER_OFF,
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
