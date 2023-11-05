/**
 * @file run.cpp
 * @author Noa Sendlhofer
 */

#include <stdexcept>
#include "pico/stdlib.h"
#include "sensor_manager.h"
#include "pico/cyw43_arch.h"
#include "motor_manager.h"
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

    std::shared_ptr<MotorManager> motor_manager = std::make_shared<MotorManager>();
    std::shared_ptr<SensorManager> sensor_manager = std::make_shared<SensorManager>(motor_manager);

    sleep_ms(2000);
    sensor_manager->calibrate();

    Controller controller(motor_manager, sensor_manager);
    const int16_t freq = 500;

    GameExecutor executor(&controller);
    controller.setExecutor(&executor);

    sleep_ms(1000);

    while(true)
    {
        controller.spin(1000.0 / freq);
        executor.checkInbox();
        sleep_ms(1000 / freq);
    }
}