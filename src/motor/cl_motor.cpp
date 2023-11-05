/**
 * @file clmotor.cpp
 * @author Noa Sendlhofer
 */

#include <valarray>
#include "cl_motor.h"

CLMotor::CLMotor(uint8_t step_pin, uint8_t dir_pin, uint8_t enc_pin_a, uint8_t enc_pin_b)
    : MotorDriver(step_pin, 0, BASE_WRAP)
    , _dir_pin(dir_pin)
{
    gpio_init(dir_pin);
    gpio_set_dir(dir_pin, GPIO_OUT);
    _setDirection(STOP);

    encoder = std::make_unique<RotaryEncoder>(enc_pin_a, enc_pin_b);
    encoder->set_rotation(0);
}

void CLMotor::setPosition(int32_t position, int32_t speed)
{
    printf("pos %li\n\r", getCurrentPosition());
    _set_position = position;
    _set_speed = speed;

    while(abs(getCurrentPosition() - _set_position) > 30)
    {
        spinController(2);
        sleep_ms(2);
    }

    printf("pos %li\n\r", getCurrentPosition());
    _setDirection(STOP);
    _integral = 0;
}

int32_t CLMotor::getCurrentPosition()
{
    return encoder->get_rotation();
}

void CLMotor::spinController(double dt)
{
    if (abs(getCurrentPosition() - _set_position) > 50)
    {
        _min_speed = MIN_SPEED;
        _max_speed = MAX_SPEED;
        _driveOpenLoop();
    }
    else
    {
        _min_speed = CTRL_MIN_SPEED;
        _max_speed = CTRL_MAX_SPEED;
        _driveClosedLoop(dt);
    }
}

void CLMotor::_driveOpenLoop()
{
    _setSpeed(_set_speed);

    if (_set_position - getCurrentPosition() > 0)
        _setDirection(FORWARD);
    else
        _setDirection(BACKWARD);
}

void CLMotor::_driveClosedLoop(double dt)
{
    int32_t position_error = getCurrentPosition() - _set_position;

    double Pout = CTRL_KP * position_error;

    _integral += int32_t(dt * position_error);
    double Iout = CTRL_KI * _integral;

    double derivative = (position_error - _last_error) / dt;
    double Dout = CTRL_KD * derivative;

    double output = Pout + Dout + Iout;

    _last_error = position_error;

    int32_t motor_out = (int32_t)output / 4;

    printf("out: %li, curr_pos %li, set_pos %li\n\r", motor_out, getCurrentPosition(), _set_position);

    _setSpeed(abs(motor_out));

    if (motor_out < 0)
        _setDirection(FORWARD);
    else
        _setDirection(BACKWARD);
}

void CLMotor::_setDirection(MotorDirection direction)
{
    switch (direction)
    {
        case FORWARD:
            _writeDirection(FORWARD);
            this->_current_dir = FORWARD;
            break;
        case BACKWARD:
            _writeDirection(BACKWARD);
            this->_current_dir = BACKWARD;
            break;
        case STOP:
            _max_speed = MAX_SPEED;
            _min_speed = MIN_SPEED;
            _setSpeed(0);
            this->_current_dir = STOP;
            break;
    }
}

void CLMotor::_writeDirection(MotorDirection direction) const
{
    switch (direction)
    {
        case FORWARD:
            gpio_put(_dir_pin, 1);
            break;
        case BACKWARD:
            gpio_put(_dir_pin, 0);
            break;
        default:
            break;
    }
}

void CLMotor::_setSpeed(int32_t speed)
{
    if (speed > _max_speed)
        speed = _max_speed;

    if (speed < _min_speed)
        speed = _min_speed;

    writePWM(speed);
}

void CLMotor::stop()
{
    _setDirection(STOP);
}
