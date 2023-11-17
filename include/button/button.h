/**
 * @file button.h
 * @author Noa Sendlhofer
 */

#ifndef PICO_MOTORS_BUTTON_H_
#define PICO_MOTORS_BUTTON_H_

#include <cstdint>
#include <ctime>
#include "hardware/watchdog.h"

#define BUTTON_PIN 26

class Button
{
public:
    explicit Button(uint8_t pin);

    static bool isPressed();

    static void callback(uint gpio, uint32_t events);
    static bool enabled;
};

#endif //PICO_MOTORS_BUTTON_H_
