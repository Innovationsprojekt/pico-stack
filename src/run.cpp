/**
 * @file run.h
 * @author Noa Sendlhofer
 */

#include "pico/stdlib.h"
#include "motor_driver.h"
#include "sensor_manager.h"

int main()
{
    Motor motor1(8,9);
    motor1.setDirection(FORWARD);
    motor1.setSpeed(200);

    SensorManager sensor_manager;
    sensor_manager.readSensor(SENSOR_B1);

    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    while (true) {
        gpio_put(LED_PIN, 1);
        sleep_ms(250);
        gpio_put(LED_PIN, 0);
        sleep_ms(250);
    }
}