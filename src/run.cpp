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
        /*
        int16_t s1 = sensor_manager->readSensor(SENSOR_F1);
        int16_t s2 = sensor_manager->readSensor(SENSOR_F2);
        int16_t s3 = sensor_manager->readSensor(SENSOR_C1);
        int16_t s4 = sensor_manager->readSensor(SENSOR_C2);
        int16_t s5 = sensor_manager->readSensor(SENSOR_B1);
        int16_t s6 = sensor_manager->readSensor(SENSOR_B2);
        printf("F1 %i, F2, %i, C1, %i, C2, %i, B1, %i, B2, %i\n\r", s1, s2, s3, s4, s5, s6);
        */
        controller.spin(1000.0 / freq);
        sleep_ms(1000 / freq);
    }
}