/**
 * @file run.h
 * @author Noa Sendlhofer
 */

#include <stdexcept>
#include "pico/stdlib.h"
#include "motor_driver.h"
#include "sensor_manager.h"
#include "pico/cyw43_arch.h"
#include "motor_manager.h"

extern "C"{
#include "ros_wrapper.h"
}

int main()
{
    stdio_init_all();

    MotorManager motor_manager;

    sleep_ms(2000);

    motor_manager.setDirection(FORWARD);
    motor_manager.setSpeed(20);

    while(true)
    {
        for(int i = 0; i<10; i++)
        {
            sleep_ms(2000);
            motor_manager.changeDirection();

            sleep_ms(2000);
            motor_manager.changeDirection();

            /*
            motor_manager.turn(90, true);
            sleep_ms(5000);
            motor_manager.turn(90, false);
            sleep_ms(5000);
             */

        }
        motor_manager.setDirection(STOP);
        sleep_ms(20000);
    }
}