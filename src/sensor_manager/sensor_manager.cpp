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
    adc2 = std::make_shared<ADCWrapper>(ADC2_I2C_ADDR, ADC2_I2C_TYPE, CLK_SPEED, ADC2_SDA_PIN, ADC2_SCL_PIN);
    //adc3 = std::make_shared<ADCWrapper>(ADC3_I2C_ADDR, ADC3_I2C_TYPE, CLK_SPEED, ADC3_SDA_PIN, ADC3_SCL_PIN);

    sensor1 = std::make_unique<Sensor>(adc1, ADSX_AIN0);
    sensor2 = std::make_unique<Sensor>(adc1, ADSX_AIN1);
    /*
    sensor3 = std::make_unique<Sensor>(adc1, ADSX_AIN2);
    sensor4 = std::make_unique<Sensor>(adc1, ADSX_AIN3);
     */
    sensor5 = std::make_unique<Sensor>(adc1, ADSX_AIN2);
    sensor6 = std::make_unique<Sensor>(adc1, ADSX_AIN3);
    /*
    sensor7 = std::make_unique<Sensor>(adc2, ADSX_AIN2);
    sensor8 = std::make_unique<Sensor>(adc2, ADSX_AIN3);
     */
    sensor9 = std::make_unique<Sensor>(adc2, ADSX_AIN0);
    sensor10 = std::make_unique<Sensor>(adc2, ADSX_AIN1);
    /*
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
            return _mapDistance(sensor5->getSensorValue(), calibration.at(2));
        case SENSOR_C2:
            return _mapDistance(sensor6->getSensorValue(), calibration.at(3));
        case SENSOR_C3:
            return sensor7->getSensorValue();
        case SENSOR_C4:
            return sensor8->getSensorValue();
        case SENSOR_B1:
            return _mapDistance(sensor9->getSensorValue(), calibration.at(4));
        case SENSOR_B2:
            return _mapDistance(sensor10->getSensorValue(), calibration.at(5));
        case SENSOR_B3:
            return sensor11->getSensorValue();
        case SENSOR_B4:
            return sensor12->getSensorValue();
        default:
            throw std::runtime_error("Invalid Sensor");
    }
}

int32_t SensorManager::readRawSensor(SensorPosition sensor)
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

int32_t SensorManager::_calcHorizontalPosition(int32_t s1, int32_t s2, int32_t s3, int32_t s4)
{
    //printf("S1: %li, S2: %li, avg: %li\n\r", s1, s2, s1-s2);
    return s2-s1;
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
            s1 = readSensor(SENSOR_C1);
            s2 = readSensor(SENSOR_C2);
            s3 = 0; //sensor7->getSensorValue();
            s4 = 0; //sensor8->getSensorValue();
            break;
        case SENSOR_ROW_BACK:
            s1 = readSensor(SENSOR_B1);
            s2 = readSensor(SENSOR_B2);
            s3 = 0; //sensor11->getSensorValue();
            s4 = 0; //sensor12->getSensorValue();
            break;
    }

    if (row == SENSOR_ROW_BACK && s1 > WHITE && s2 > WHITE)
    {
        motor_manager->setDirection(STOP);
        throw std::runtime_error("END OF TRACK");
    }

    if (s1 <= BLACK && s2 <= BLACK)
        return 0;
    else
        return _calcHorizontalPosition(s1, s2, s3, s4);
}

void SensorManager::calibrate()
{
    sleep_ms(2000);
    motor_manager->turn(90, LEFT);

    int32_t B1b = readRawSensor(SENSOR_B1);
    int32_t B2b = readRawSensor(SENSOR_B2);

    motor_manager->creepDistance(8, BACKWARD);

    int32_t B1w = readRawSensor(SENSOR_B1);
    int32_t B2w = readRawSensor(SENSOR_B2);
    int32_t C1b = readRawSensor(SENSOR_C1);
    int32_t C2b = readRawSensor(SENSOR_C2);
    int32_t F1w = readRawSensor(SENSOR_F1);
    int32_t F2w = readRawSensor(SENSOR_F2);

    motor_manager->creepDistance(7, BACKWARD);

    int32_t F1b = readRawSensor(SENSOR_F1);
    int32_t F2b = readRawSensor(SENSOR_F2);
    int32_t C1w = readRawSensor(SENSOR_C1);
    int32_t C2w = readRawSensor(SENSOR_C2);

    motor_manager->creepDistance(15, FORWARD);
    motor_manager->turn(90, RIGHT);

    calibration.emplace_back(F1b, F1w);
    calibration.emplace_back(F2b, F2w);
    calibration.emplace_back(C1b, C1w);
    calibration.emplace_back(C2b, C2w);
    calibration.emplace_back(B1b, B1w);
    calibration.emplace_back(B2b, B2w);

    printf("F1 w: %li, F1 b: %li, F2 w: %li, F2 b: %li\n\r", calibration.at(0).second, calibration.at(0).first, calibration.at(1).second, calibration.at(1).first);
    printf("C1 w: %li, C1 b: %li, C2 w: %li, C2 b: %li\n\r", calibration.at(2).second, calibration.at(2).first, calibration.at(3).second, calibration.at(3).first);
    printf("B1 w: %li, B1 b: %li, B2 w: %li, B2 b: %li\n\r", calibration.at(4).second, calibration.at(4).first, calibration.at(5).second, calibration.at(5).first);
}

int32_t SensorManager::_mapDistance(int32_t s, std::pair<int32_t, int32_t> calib)
{
    double cb = s-calib.first;
    double wb = calib.second-calib.first;
    double div = cb/wb*1000.0;
    auto ret = (int32_t)div;

    return ret;
}
