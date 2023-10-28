/**
 * @file sensor_manager.cpp
 * @author Noa Sendlhofer
 */

#include <stdexcept>
#include <utility>
#include "sensor_manager.h"

SensorManager::SensorManager(std::shared_ptr<MotorManager> motor_manager) : motor_manager(std::move(motor_manager))
{

    adc1 = std::make_shared<ADCWrapper>(ADC1_I2C_ADDR, ADC1_I2C_TYPE, CLK_SPEED, ADC1_SDA_PIN, ADC1_SCL_PIN);
    /*
    adc2 = std::make_shared<ADCWrapper>(ADC2_I2C_ADDR, ADC2_I2C_TYPE, CLK_SPEED, ADC2_SDA_PIN, ADC2_SCL_PIN);
    adc3 = std::make_shared<ADCWrapper>(ADC3_I2C_ADDR, ADC3_I2C_TYPE, CLK_SPEED, ADC3_SDA_PIN, ADC3_SCL_PIN);
    */
    sensor1 = std::make_unique<Sensor>(adc1, ADSX_AIN0);
    sensor2 = std::make_unique<Sensor>(adc1, ADSX_AIN1);
    /*
    sensor3 = std::make_unique<Sensor>(adc1, ADSX_AIN2);
    sensor4 = std::make_unique<Sensor>(adc1, ADSX_AIN3);
    sensor5 = std::make_unique<Sensor>(adc2, ADSX_AIN0);
    sensor6 = std::make_unique<Sensor>(adc2, ADSX_AIN1);
    sensor7 = std::make_unique<Sensor>(adc2, ADSX_AIN2);
    sensor8 = std::make_unique<Sensor>(adc2, ADSX_AIN3);
    sensor9 = std::make_unique<Sensor>(adc3, ADSX_AIN0);
    sensor10 = std::make_unique<Sensor>(adc3, ADSX_AIN1);
    sensor11 = std::make_unique<Sensor>(adc3, ADSX_AIN2);
    sensor12 = std::make_unique<Sensor>(adc3, ADSX_AIN3);
     */
}

int32_t SensorManager::readSensor(SensorPosition sensor)
{
    switch (sensor)
    {
        case SENSOR_F1:
            return _mapDistance(sensor1->getSensorValue(), calibration.at(0));
        case SENSOR_F2:
            return _mapDistance(sensor2->getSensorValue(), calibration.at(1));
        case SENSOR_F3:
            return sensor3->getSensorValue();
        case SENSOR_F4:
            return sensor4->getSensorValue();
        case SENSOR_C1:
            return sensor5->getSensorValue();
        case SENSOR_C2:
            return sensor6->getSensorValue();
        case SENSOR_C3:
            return sensor7->getSensorValue();
        case SENSOR_C4:
            return sensor8->getSensorValue();
        case SENSOR_B1:
            return sensor9->getSensorValue();
        case SENSOR_B2:
            return sensor10->getSensorValue();
        case SENSOR_B3:
            return sensor11->getSensorValue();
        case SENSOR_B4:
            return sensor12->getSensorValue();
        default:
            throw std::runtime_error("Invalid Sensor");
    }
}

int32_t SensorManager::_calcHorizontalPosition(int32_t s1, int32_t s2, int32_t s3, int32_t s4)
{
    //printf("S1: %li, S2: %li, avg: %li\n\r", s1, s2, s1-s2);
    return s1-s2;
}

int32_t SensorManager::getHorizontalPosition(SensorRow row)
{
    int32_t s1, s2, s3, s4;

    switch (row)
    {
        case SENSOR_ROW_FRONT:
            s1 = readSensor(SENSOR_F1);
            s2 = readSensor(SENSOR_F2);
            s3 = 0;
            s4 = 0;
            break;
        case SENSOR_ROW_CENTER:
            s1 = sensor5->getSensorValue();
            s2 = sensor6->getSensorValue();
            s3 = sensor7->getSensorValue();
            s4 = sensor8->getSensorValue();
            break;
        case SENSOR_ROW_BACK:
            s1 = sensor9->getSensorValue();
            s2 = sensor10->getSensorValue();
            s3 = sensor11->getSensorValue();
            s4 = sensor12->getSensorValue();
            break;
    }

    if (s1 > WHITE && s2 > WHITE)
    {
        motor_manager->setDirection(STOP);
        throw std::runtime_error("END OF TRACK");
    }

    return _calcHorizontalPosition(s1, s2, s3, s4);
}

void SensorManager::calibrate()
{
    sleep_ms(2000);
    int16_t s1b = sensor1->getSensorValue();
    int16_t s2b = sensor2->getSensorValue();
    motor_manager->turn(45, DIR_LEFT);
    int16_t s1w = sensor1->getSensorValue();
    int16_t s2w = sensor2->getSensorValue();
    motor_manager->turn(45, DIR_RIGHT);

    calibration.emplace_back(s1b, s1w);
    calibration.emplace_back(s2b, s2w);

    printf("S1 w: %i, S1 b: %i, S2 w: %i, S2 b: %i\n\r", calibration.at(0).second, calibration.at(0).first, calibration.at(1).second, calibration.at(1).first);
}

int32_t SensorManager::_mapDistance(int16_t s, std::pair<int16_t, int16_t> calib)
{
    double cb = s-calib.first;
    double wb = calib.second-calib.first;
    double div = cb/wb*1000.0;
    auto ret = (int32_t)div;

    //printf("raw: %i, c-b: %f, w-b: %f, fl: %f, ret %li\n\r", s, cb, wb, div, ret);

    return ret;
}
