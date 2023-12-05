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
#include "motor_manager.h"
#include "enum_definitions.h"

#define CLK_SPEED 100

#define ADC1_I2C_ADDR ADSX_ADDRESS_GND
#define ADC1_I2C_TYPE i2c0
#define ADC1_SDA_PIN 16
#define ADC1_SCL_PIN 17

#define ADC2_I2C_ADDR ADSX_ADDRESS_VDD
#define ADC2_I2C_TYPE i2c0
#define ADC2_SDA_PIN 16
#define ADC2_SCL_PIN 17

#define ADC3_I2C_ADDR ADSX_ADDRESS_SCLK
#define ADC3_I2C_TYPE i2c0
#define ADC3_SDA_PIN 16
#define ADC3_SCL_PIN 17

#define WHITE 980
#define BLACK 50
#define ON_LINE 150

class SensorManager
{
public:
    SensorManager(std::shared_ptr<MotorManager> motor_manager);
    int32_t readSensor(SensorPosition sensor);
    int32_t readRawSensor(SensorPosition sensor);
    int32_t getHorizontalPosition(SensorRow row);
    int32_t getFullHorizontalPosition(SensorRow row);
    void calibrate();
    bool isCalibrated();

private:
    static int32_t _calcHorizontalPosition(int32_t s1, int32_t s2, int32_t s3, int32_t s4);
    static int32_t _mapDistance(int32_t s, std::pair<int32_t, int32_t> calib);

    std::shared_ptr<MotorManager> motor_manager;

    std::vector<std::pair<int32_t, int32_t>> calibration;

    std::shared_ptr<ADCWrapper> adc1;
    std::shared_ptr<ADCWrapper> adc2;
    std::shared_ptr<ADCWrapper> adc3;

    std::unique_ptr<Sensor> sensorFLI;
    std::unique_ptr<Sensor> sensorFRI;
    std::unique_ptr<Sensor> sensorFLO;
    std::unique_ptr<Sensor> sensorFRO;
    std::unique_ptr<Sensor> sensorCLI;
    std::unique_ptr<Sensor> sensorCRI;
    std::unique_ptr<Sensor> sensorCLO;
    std::unique_ptr<Sensor> sensorCRO;
    std::unique_ptr<Sensor> sensorBLI;
    std::unique_ptr<Sensor> sensorBRI;
    std::unique_ptr<Sensor> sensorBLO;
    std::unique_ptr<Sensor> sensorBRO;

    bool calibrated = false;
};


#endif //PICO_MOTORS_SENSOR_MANAGER_H_
