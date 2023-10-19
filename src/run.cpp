/**
 * @file run.h
 * @author Noa Sendlhofer
 */

#include <stdexcept>
#include "pico/stdlib.h"
#include "motor_driver.h"
#include "sensor_manager.h"
#include "pico/cyw43_arch.h"

extern "C"{
#include "ros_wrapper.h"
}

int main()
{
    stdio_init_all();
    if (cyw43_arch_init())
        throw std::runtime_error("WIFI failed");

    ROSWrapper();

    while (true) {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        sleep_ms(1000);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        sleep_ms(250);
    }
}