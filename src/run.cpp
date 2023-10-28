/**
 * @file run.cpp
 * @author Noa Sendlhofer
 */

#include <stdexcept>
#include "pico/stdlib.h"
#include "motor_driver.h"
#include "sensor_manager.h"
#include "pico/cyw43_arch.h"
#include "motor_manager.h"
#include "controller.h"

extern "C" {
#include "ros_wrapper.h"
}

#include "pico/multicore.h"

#include "hardware/adc.h"

void core1_main()
{
    MotorManager motor_manager;
    motor_manager.setSpeed(0);

    while(true)
    {
        for (int i = 0; i < 100; i++)
        {
            multicore_fifo_push_blocking(motor_manager.motor1->getCurrentSpeed());
            motor_manager.setSpeed(i);
            sleep_ms(100);
        }
        for (int i = 100; i > 0; i--)
        {
            multicore_fifo_push_blocking(motor_manager.motor1->getCurrentSpeed());
            motor_manager.setSpeed(i);
            sleep_ms(100);
        }
    }
}

void print(const char* text)
{
    char * add = "\n\r";
    printf("%s", strcat(add, text));
}

int main()
{
    stdio_init_all();

    std::shared_ptr<MotorManager> motor_manager = std::make_shared<MotorManager>();
    std::shared_ptr<SensorManager> sensor_manager = std::make_shared<SensorManager>(motor_manager);
    sensor_manager->calibrate();
    sleep_ms(2000);

    Controller controller(motor_manager, sensor_manager);
    motor_manager->setDirection(BACKWARD);

    int16_t freq = 500;

    while(true)
    {
        controller.spin(1000.0 / freq);
        sleep_ms(1000 / freq);
    }
}