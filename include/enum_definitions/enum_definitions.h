/**
 * @file enum_definitions.h
 * @author Noa Sendlhofer
 */

#ifndef PICO_MOTORS_ENUM_DEFINITIONS_H_
#define PICO_MOTORS_ENUM_DEFINITIONS_H_

#include <vector>

#define GAME_PLAN_TEST

#ifndef GAME_PLAN_TEST
    //#define GAME_PLAN_UNLOAD
    #define GAME_PLAN_GOAL
    //#define SAFE_MODE
#endif

#define UNLOAD_DISTANCE 5

//#define ENABLE_SENSOR_CALIB
//#define ENABLE_END_OF_TRACK
#define ENABLE_PICKUP
#define ENABLE_UNLOAD
#define ENABLE_MIXER
#define PRE_CALIBRATION

#define SERVO_SPEED 4

struct PIDConfiguration
{
    double kp;
    double kd;
    double ki;
    uint16_t speed;
    uint16_t offset;
};

enum DriveSpeed
{
    SPEED_STRAIGHT,
    SPEED_CURVE,
    SPEED_GATE,
};

enum LineSide
{
    DETECT_LEFT,
    DETECT_RIGHT,
    DETECT_UNLOAD,
};

enum PickUpSide
{
    PICKUP_LEFT,
    PICKUP_LEFT_SHIELD,
    PICKUP_LEFT_OUT_CURVE,
    PICKUP_LEFT_IN_CURVE,
    PICKUP_RIGHT,
    PICKUP_RIGHT_SHIELD,
    PICKUP_RIGHT_OUT_CURVE,
    PICKUP_RIGHT_IN_CURVE,
};

enum TurnDirection
{
    LEFT,
    RIGHT,
};

enum MotorDirection
{
    FORWARD,
    BACKWARD,
    STOP,
};

enum LineType
{
    STRAIGHT,
    CURVE_LEFT,
    CURVE_RIGHT,
};

enum GameMessage
{
    NOTIFY_READY,
    NOTIFY_CALIBRATE,
    NOTIFY_LINE,
    NOTIFY_ALIGN,
    NOTIFY_TRASH,
    NOTIFY_RESUME,
    NOTIFY_UNLOAD,
    NOTIFY_WIGGLE,
    REQUEST_CALIBRATE,
    REQUEST_DRIVE_STRAIGHT,
    REQUEST_DRIVE_CURVE,
    REQUEST_DRIVE_GATE,
    REQUEST_LINE_LEFT,
    REQUEST_LINE_RIGHT,
    REQUEST_LINE_UNLOAD,
    REQUEST_ALIGN_STRAIGHT,
    REQUEST_ALIGN_CURVE_LEFT,
    REQUEST_ALIGN_CURVE_RIGHT,
    REQUEST_RESUME_CURVE_LEFT,
    REQUEST_RESUME_CURVE_RIGHT,
    REQUEST_PICKUP_LEFT,
    REQUEST_PICKUP_LEFT_SHIELD,
    REQUEST_PICKUP_LEFT_OUT_CURVE,
    REQUEST_PICKUP_LEFT_IN_CURVE,
    REQUEST_PICKUP_RIGHT,
    REQUEST_PICKUP_RIGHT_SHIELD,
    REQUEST_PICKUP_RIGHT_OUT_CURVE,
    REQUEST_PICKUP_RIGHT_IN_CURVE,
    REQUEST_UNLOAD,
    REQUEST_MIXER_ON,
    REQUEST_MIXER_OFF,
    REQUEST_WIGGLE
};

enum GameItems
{
    CALIBRATE,
    DRIVE_STRAIGHT,
    DRIVE_CURVE,
    DRIVE_GATE,
    LINE_LEFT,
    LINE_RIGHT,
    LINE_UNLOAD,
    ALIGN_STRAIGHT,
    ALIGN_CURVE_LEFT,
    ALIGN_CURVE_RIGHT,
    RESUME_CURVE_LEFT,
    RESUME_CURVE_RIGHT,
    TRASH_LEFT,
    TRASH_LEFT_SHIELD,
    TRASH_LEFT_OUT_CURVE,
    TRASH_LEFT_IN_CURVE,
    TRASH_RIGHT,
    TRASH_RIGHT_SHIELD,
    TRASH_RIGHT_OUT_CURVE,
    TRASH_RIGHT_IN_CURVE,
    UNLOAD,
    MIXER_ON,
    MIXER_OFF,
    WIGGLE,
    WIGGLE_WAIT,
};

enum SensorPosition
{
    SENSOR_FLI,
    SENSOR_FRI,
    SENSOR_FLO,
    SENSOR_FRO,
    SENSOR_CLI,
    SENSOR_CRI,
    SENSOR_CLO,
    SENSOR_CRO,
    SENSOR_BLI,
    SENSOR_BRI,
    SENSOR_BLO,
    SENSOR_BRO,
};

enum SensorRow
{
    SENSOR_ROW_FRONT,
    SENSOR_ROW_CENTER,
    SENSOR_ROW_BACK,
};

#endif //PICO_MOTORS_ENUM_DEFINITIONS_H_
