/**
 * @file run.cpp
 * @author Noa Sendlhofer
 */

#include <stdexcept>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "controller.h"
#include "game_executor.h"
#include "game_controller.h"

extern "C" {
#include "ros_wrapper.h"
}


void core1_main()
{
    GameController game;

    while(true)
    {
        game.checkInbox();
    }
}

int main()
{
    stdio_init_all();

    multicore_launch_core1(core1_main);

    sleep_ms(2000); //TODO implement button press

    Controller controller;
    const int16_t freq = 500;

    GameExecutor executor(&controller);
    controller.setExecutor(&executor);

    sleep_ms(1000);

    controller.start();

    while(true)
    {
        controller.spin(1000.0 / freq);
        executor.checkInbox();
        sleep_ms(1000 / freq);
    }
}