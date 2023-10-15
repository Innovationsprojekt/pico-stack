/**
 * @file sensor_manager.cpp
 * @author Noa Sendlhofer
 */

#include <stdexcept>
#include "sensor_manager.h"

SensorManager::SensorManager()
{
    adc1 = std::make_shared<ADCWrapper>(ADC1_I2C_ADDR, ADC1_I2C_TYPE, CLK_SPEED, ADC1_SDA_PIN, ADC1_SCL_PIN);
    adc2 = std::make_shared<ADCWrapper>(ADC2_I2C_ADDR, ADC2_I2C_TYPE, CLK_SPEED, ADC2_SDA_PIN, ADC2_SCL_PIN);
    adc3 = std::make_shared<ADCWrapper>(ADC3_I2C_ADDR, ADC3_I2C_TYPE, CLK_SPEED, ADC3_SDA_PIN, ADC3_SCL_PIN);

    sensor1 = std::make_unique<Sensor>(adc1, ADSX_AIN0);
    sensor2 = std::make_unique<Sensor>(adc1, ADSX_AIN1);
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
}

int16_t SensorManager::readSensor(SensorPosition sensor)
{
    switch (sensor)
    {
        case SENSOR_F1:
            return sensor1->getSensorValue();
        case SENSOR_F2:
            return sensor2->getSensorValue();
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

int16_t SensorManager::_calcHorizontalPosition(int16_t s1, int16_t s2, int16_t s3, int16_t s4)
{
    return 0; //TODO implement smart algorithm ^^
}

int16_t SensorManager::getHorizontalPosition(SensorRow row)
{
    int16_t s1, s2, s3, s4;

    switch (row)
    {
        case SENSOR_ROW_FRONT:
            s1 = sensor1->getSensorValue();
            s2 = sensor2->getSensorValue();
            s3 = sensor3->getSensorValue();
            s4 = sensor4->getSensorValue();
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

    return _calcHorizontalPosition(s1, s2, s3, s4);
}
