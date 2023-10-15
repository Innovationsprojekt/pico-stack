/**
 * @file sensor_manager.h
 * @author Noa Sendlhofer
 */

#ifndef PICO_MOTORS_SENSOR_MANAGER_H_
#define PICO_MOTORS_SENSOR_MANAGER_H_

#include <memory>
#include "adc_wrapper.h"
#include "sensor.h"

#define CLK_SPEED 100

#define ADC1_I2C_ADDR ADSX_ADDRESS_GND
#define ADC1_I2C_TYPE i2c1
#define ADC1_SDA_PIN 18
#define ADC1_SCL_PIN 19

#define ADC2_I2C_ADDR ADSX_ADDRESS_VDD
#define ADC2_I2C_TYPE i2c1
#define ADC2_SDA_PIN 18
#define ADC2_SCL_PIN 19

#define ADC3_I2C_ADDR ADSX_ADDRESS_SCLK
#define ADC3_I2C_TYPE i2c1
#define ADC3_SDA_PIN 18
#define ADC3_SCL_PIN 19

enum SensorPosition
{
    SENSOR_F1 = 0,
    SENSOR_F2 = 1,
    SENSOR_F3 = 2,
    SENSOR_F4 = 3,
    SENSOR_C1 = 4,
    SENSOR_C2 = 5,
    SENSOR_C3 = 6,
    SENSOR_C4 = 7,
    SENSOR_B1 = 8,
    SENSOR_B2 = 9,
    SENSOR_B3 = 10,
    SENSOR_B4 = 11,
};

enum SensorRow
{
    SENSOR_ROW_FRONT = 0,
    SENSOR_ROW_CENTER = 1,
    SENSOR_ROW_BACK = 2,
};

class SensorManager
{
public:
    SensorManager();
    int16_t readSensor(SensorPosition sensor);
    int16_t getHorizontalPosition(SensorRow row);

private:
    int16_t _calcHorizontalPosition(int16_t s1, int16_t s2, int16_t s3, int16_t s4);

    std::shared_ptr<ADCWrapper> adc1;
    std::shared_ptr<ADCWrapper> adc2;
    std::shared_ptr<ADCWrapper> adc3;

    std::unique_ptr<Sensor> sensor1;
    std::unique_ptr<Sensor> sensor2;
    std::unique_ptr<Sensor> sensor3;
    std::unique_ptr<Sensor> sensor4;
    std::unique_ptr<Sensor> sensor5;
    std::unique_ptr<Sensor> sensor6;
    std::unique_ptr<Sensor> sensor7;
    std::unique_ptr<Sensor> sensor8;
    std::unique_ptr<Sensor> sensor9;
    std::unique_ptr<Sensor> sensor10;
    std::unique_ptr<Sensor> sensor11;
    std::unique_ptr<Sensor> sensor12;
};


#endif //PICO_MOTORS_SENSOR_MANAGER_H_
