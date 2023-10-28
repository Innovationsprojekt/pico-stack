/**
 * @file sensor_manager.h
 * @author Noa Sendlhofer
 */

#ifndef PICO_MOTORS_SENSOR_MANAGER_H_
#define PICO_MOTORS_SENSOR_MANAGER_H_

#include <memory>
#include <vector>
#include "adc_wrapper.h"
#include "sensor.h"
#include "motor_manager.h";

#define CLK_SPEED 100

#define ADC1_I2C_ADDR ADSX_ADDRESS_GND
#define ADC1_I2C_TYPE i2c0
#define ADC1_SDA_PIN 0
#define ADC1_SCL_PIN 1

#define ADC2_I2C_ADDR ADSX_ADDRESS_VDD
#define ADC2_I2C_TYPE i2c1
#define ADC2_SDA_PIN 18
#define ADC2_SCL_PIN 19

#define ADC3_I2C_ADDR ADSX_ADDRESS_SCLK
#define ADC3_I2C_TYPE i2c1
#define ADC3_SDA_PIN 18
#define ADC3_SCL_PIN 19

#define WHITE 650
#define BLACK 100

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
    SensorManager(std::shared_ptr<MotorManager> motor_manager);
    int32_t readSensor(SensorPosition sensor);
    int32_t getHorizontalPosition(SensorRow row);
    void calibrate();

private:
    int32_t _calcHorizontalPosition(int32_t s1, int32_t s2, int32_t s3, int32_t s4);
    int32_t _mapDistance(int16_t s, std::pair<int16_t, int16_t> calib);

    std::shared_ptr<MotorManager> motor_manager;

    std::vector<std::pair<int16_t, int16_t>> calibration;

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
