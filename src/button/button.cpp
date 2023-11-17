/**
 * @file button.cpp
 * @author Noa Sendlhofer
 */

#include "button.h"
#include "hardware/gpio.h"

void software_reset()
{
    watchdog_enable(1, true);
    while(1);
}

bool Button::enabled = false;
Button button(BUTTON_PIN);

Button::Button(uint8_t pin)
{
    gpio_set_irq_enabled_with_callback(pin, GPIO_IRQ_EDGE_RISE, true, &callback);
}

void Button::callback(uint gpio, uint32_t events)
{
    if (enabled)
        software_reset();
    else
        enabled = true;
}

bool Button::isPressed()
{
    return enabled;
}
