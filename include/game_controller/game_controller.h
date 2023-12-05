/**
 * @file game_controller.h
 * @author Noa Sendlhofer
 */

#ifndef PICO_MOTORS_GAME_CONTROLLER_H_
#define PICO_MOTORS_GAME_CONTROLLER_H_

#include <vector>
#include "communication_manager.h"
#include "enum_definitions.h"

#define LINE_WAIT_TIME 1500

#define CALIBRATION_TIME 2000
#define LINE_TIME 1000
#define PICKUP_TIME 4000
#define UNLOAD_TIME 10000

#ifdef GAME_PLAN_TEST
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

           WIGGLE,

           DRIVE_CURVE,
           LINE_LEFT,
           ALIGN_CURVE_LEFT,
           TRASH_RIGHT_CURVE,

           DRIVE_CURVE,
           LINE_LEFT,
           ALIGN_CURVE_LEFT,
           TRASH_LEFT,

           WIGGLE_WAIT,

           DRIVE_STRAIGHT,
           LINE_LEFT,
           UNLOAD,
           MIXER_OFF
        };
#endif

#ifdef GAME_PLAN_GOAL
const std::vector<GameItems> game_plan
        = {CALIBRATE,

           //STRAIGHT 1
           DRIVE_STRAIGHT,
           LINE_LEFT,
           ALIGN_STRAIGHT,
           MIXER_ON,
           TRASH_LEFT,

           DRIVE_STRAIGHT,
           LINE_RIGHT,
           ALIGN_STRAIGHT,
           TRASH_RIGHT,

           //WIGGLE,

           DRIVE_STRAIGHT,
           LINE_LEFT,
           ALIGN_STRAIGHT,
           TRASH_LEFT,

           WIGGLE,

           //CURVE 1
           DRIVE_CURVE,
           LINE_RIGHT,
           ALIGN_CURVE_RIGHT,
           TRASH_RIGHT_IN_CURVE,
           RESUME_CURVE_RIGHT,

           //STRAIGHT 2
           DRIVE_STRAIGHT,
           LINE_LEFT,
           ALIGN_STRAIGHT,
           TRASH_LEFT,

           WIGGLE,

           DRIVE_STRAIGHT,
           LINE_RIGHT,

           DRIVE_STRAIGHT,
           LINE_LEFT,
           /*
           ALIGN_STRAIGHT,
           TRASH_LEFT,
            */

           //WIGGLE,

           //CURVE 2
           DRIVE_CURVE,
           LINE_LEFT,
           ALIGN_CURVE_LEFT,
           TRASH_RIGHT_OUT_CURVE,

           DRIVE_CURVE,
           LINE_LEFT,
           ALIGN_CURVE_LEFT,
           TRASH_LEFT_IN_CURVE,
           RESUME_CURVE_LEFT,

           WIGGLE_WAIT,
           DRIVE_STRAIGHT,
           LINE_LEFT,

           //UNLOAD
           UNLOAD,
           DRIVE_CURVE,
           LINE_RIGHT,


           //CURVE 3
           DRIVE_CURVE,
           LINE_RIGHT,
           ALIGN_CURVE_RIGHT,
           TRASH_LEFT_OUT_CURVE,

           DRIVE_CURVE,
           LINE_RIGHT,
           ALIGN_CURVE_RIGHT,
           TRASH_RIGHT_IN_CURVE,
           RESUME_CURVE_RIGHT,

           ALIGN_CURVE_RIGHT,
           TRASH_LEFT_OUT_CURVE,

           WIGGLE_WAIT,

           //STRAIGHT 3
           DRIVE_STRAIGHT,
           LINE_RIGHT,
           ALIGN_STRAIGHT,
           TRASH_LEFT,


           //WIGGLE,

           DRIVE_STRAIGHT,
           LINE_RIGHT,

           DRIVE_STRAIGHT,
           LINE_RIGHT,
           /*
           ALIGN_STRAIGHT,
           TRASH_LEFT,
            */

           WIGGLE,

           //CURVE 4
           DRIVE_CURVE,
           LINE_LEFT,
           ALIGN_CURVE_LEFT,
           TRASH_RIGHT_OUT_CURVE,

           DRIVE_CURVE,
           LINE_LEFT,
           ALIGN_CURVE_LEFT,
           TRASH_LEFT_IN_CURVE,
           RESUME_CURVE_LEFT,

           ALIGN_CURVE_LEFT,
           TRASH_RIGHT_OUT_CURVE,

           WIGGLE_WAIT,

           DRIVE_STRAIGHT,
           LINE_LEFT,
           UNLOAD,
           MIXER_OFF,
        };
#endif

#ifdef GAME_PLAN_UNLOAD
const std::vector<GameItems> game_plan
        = {CALIBRATE,

           //STRAIGHT 1
           DRIVE_STRAIGHT,
           LINE_LEFT,
           ALIGN_STRAIGHT,
           MIXER_ON,
           TRASH_LEFT,

           DRIVE_STRAIGHT,
           LINE_RIGHT,
           ALIGN_STRAIGHT,
           TRASH_RIGHT,

           //WIGGLE,

           DRIVE_STRAIGHT,
           LINE_LEFT,
           ALIGN_STRAIGHT,
           TRASH_LEFT,

           WIGGLE,

           //CURVE 1
           DRIVE_CURVE,
           LINE_RIGHT,
           ALIGN_CURVE_RIGHT,
           TRASH_RIGHT_IN_CURVE,
           RESUME_CURVE_RIGHT,

           //STRAIGHT 2
           DRIVE_STRAIGHT,
           LINE_LEFT,
           ALIGN_STRAIGHT,
           TRASH_LEFT,

           WIGGLE,

           DRIVE_STRAIGHT,
           LINE_RIGHT,

           DRIVE_STRAIGHT,
           LINE_LEFT,
           ALIGN_STRAIGHT,
           TRASH_LEFT,

           //WIGGLE,

           //CURVE 2
           DRIVE_CURVE,
           LINE_LEFT,
           ALIGN_CURVE_LEFT,
           TRASH_RIGHT_OUT_CURVE,

           DRIVE_CURVE,
           LINE_LEFT,
           ALIGN_CURVE_LEFT,
           TRASH_LEFT_IN_CURVE,
           RESUME_CURVE_LEFT,

           WIGGLE_WAIT,
           DRIVE_STRAIGHT,
           LINE_LEFT,

           //UNLOAD
           UNLOAD,
           DRIVE_CURVE,
           LINE_RIGHT,


           //CURVE 3
           DRIVE_CURVE,
           LINE_LEFT,
           ALIGN_CURVE_RIGHT,
           TRASH_LEFT_OUT_CURVE,

           DRIVE_CURVE,
           LINE_RIGHT,
           ALIGN_CURVE_RIGHT,
           TRASH_RIGHT_IN_CURVE,
           RESUME_CURVE_RIGHT,

           ALIGN_CURVE_RIGHT,
           TRASH_LEFT_OUT_CURVE,

           WIGGLE_WAIT,

           //STRAIGHT 3
           DRIVE_STRAIGHT,
           LINE_RIGHT,
           ALIGN_STRAIGHT,
           TRASH_LEFT,


           //WIGGLE,

           DRIVE_STRAIGHT,
           LINE_RIGHT,

           DRIVE_STRAIGHT,
           LINE_RIGHT,
           ALIGN_STRAIGHT,
           TRASH_LEFT,

           WIGGLE,

           //CURVE 4
           DRIVE_CURVE,
           LINE_LEFT,
           ALIGN_CURVE_LEFT,
           TRASH_RIGHT_OUT_CURVE,

           DRIVE_CURVE,
           LINE_LEFT,
           ALIGN_CURVE_LEFT,
           TRASH_LEFT_IN_CURVE,
           RESUME_CURVE_LEFT,

           ALIGN_CURVE_LEFT,
           TRASH_RIGHT_OUT_CURVE,

           WIGGLE_WAIT,

           DRIVE_STRAIGHT,
           LINE_LEFT,
           UNLOAD,
           //MIXER_OFF,
        };
#endif

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
