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
    adc3 = std::make_shared<ADCWrapper>(ADC3_I2C_ADDR, ADC3_I2C_TYPE, CLK_SPEED, ADC3_SDA_PIN, ADC3_SCL_PIN);

    sensorFLO = std::make_unique<Sensor>(adc2, ADSX_AIN3);
    sensorFLI = std::make_unique<Sensor>(adc2, ADSX_AIN2);
    sensorFRI = std::make_unique<Sensor>(adc2, ADSX_AIN1);
    sensorFRO = std::make_unique<Sensor>(adc2, ADSX_AIN0);

    sensorCLO = std::make_unique<Sensor>(adc1, ADSX_AIN0);
    sensorCLI = std::make_unique<Sensor>(adc1, ADSX_AIN1);
    sensorCRI = std::make_unique<Sensor>(adc1, ADSX_AIN2);
    sensorCRO = std::make_unique<Sensor>(adc1, ADSX_AIN3);

    sensorBLO = std::make_unique<Sensor>(adc3, ADSX_AIN3);
    sensorBLI = std::make_unique<Sensor>(adc3, ADSX_AIN2);
    sensorBRI = std::make_unique<Sensor>(adc3, ADSX_AIN1);
    sensorBRO = std::make_unique<Sensor>(adc3, ADSX_AIN0);
}

int32_t SensorManager::readSensor(SensorPosition sensor)
{
    if (!calibrated)
        throw std::runtime_error("Sensors not yet calibrated!");

    switch (sensor)
    {
        case SENSOR_FLO:
            return _mapDistance(sensorFLO->getSensorValue(), calibration.at(0));
        case SENSOR_FLI:
            return _mapDistance(sensorFLI->getSensorValue(), calibration.at(1));
        case SENSOR_FRI:
            return _mapDistance(sensorFRI->getSensorValue(), calibration.at(2));
        case SENSOR_FRO:
            return _mapDistance(sensorFRO->getSensorValue(), calibration.at(3));
        case SENSOR_CLO:
            return _mapDistance(sensorCLO->getSensorValue(), calibration.at(4));
        case SENSOR_CLI:
            return _mapDistance(sensorCLI->getSensorValue(), calibration.at(5));
        case SENSOR_CRI:
            return _mapDistance(sensorCRI->getSensorValue(), calibration.at(6));
        case SENSOR_CRO:
            return _mapDistance(sensorCRO->getSensorValue(), calibration.at(7));
        case SENSOR_BLO:
            return _mapDistance(sensorBLO->getSensorValue(), calibration.at(8));
        case SENSOR_BLI:
            return _mapDistance(sensorBLI->getSensorValue(), calibration.at(9));
        case SENSOR_BRI:
            return _mapDistance(sensorBRI->getSensorValue(), calibration.at(10));
        case SENSOR_BRO:
            return _mapDistance(sensorBRO->getSensorValue(), calibration.at(11));
        default:
            throw std::runtime_error("Invalid Sensor");
    }
}

int32_t SensorManager::readRawSensor(SensorPosition sensor)
{
    switch (sensor)
    {
        case SENSOR_FLI:
            return sensorFLI->getSensorValue();
        case SENSOR_FRI:
            return sensorFRI->getSensorValue();
        case SENSOR_FLO:
            return sensorFLO->getSensorValue();
        case SENSOR_FRO:
            return sensorFRO->getSensorValue();
        case SENSOR_CLI:
            return sensorCLI->getSensorValue();
        case SENSOR_CRI:
            return sensorCRI->getSensorValue();
        case SENSOR_CLO:
            return sensorCLO->getSensorValue();
        case SENSOR_CRO:
            return sensorCRO->getSensorValue();
        case SENSOR_BLI:
            return sensorBLI->getSensorValue();
        case SENSOR_BRI:
            return sensorBRI->getSensorValue();
        case SENSOR_BLO:
            return sensorBLO->getSensorValue();
        case SENSOR_BRO:
            return sensorBRO->getSensorValue();
        default:
            throw std::runtime_error("Invalid Sensor");
    }
}

int32_t SensorManager::_calcHorizontalPosition(int32_t li, int32_t ri, int32_t lo, int32_t ro)
{
    //printf("S1: %li, S2: %li, avg: %li\n\r", s1, s2, s1-s2);
    //return ri + ro - li - lo; //TODO check
    return ri - li; //TODO check
}

int32_t SensorManager::getHorizontalPosition(SensorRow row)
{
    int32_t li, ri, lo, ro;

    switch (row)
    {
        case SENSOR_ROW_FRONT:
            li = readSensor(SENSOR_FLI);
            ri = readSensor(SENSOR_FRI);
            lo = readSensor(SENSOR_FLO);
            ro = readSensor(SENSOR_FRO);
            break;
        case SENSOR_ROW_CENTER:
            li = readSensor(SENSOR_CLI);
            ri = readSensor(SENSOR_CRI);
            lo = readSensor(SENSOR_CLO);
            ro = readSensor(SENSOR_CRO);
            break;
        case SENSOR_ROW_BACK:
            li = readSensor(SENSOR_BLI);
            ri = readSensor(SENSOR_BRI);
            lo = readSensor(SENSOR_BLO);
            ro = readSensor(SENSOR_BRO);
            break;
    }

#ifdef ENABLE_END_OF_TRACK
    if (row == SENSOR_ROW_BACK && li > WHITE && ri > WHITE)
    {
        motor_manager->setDirection(STOP);
        printf("END OF TRACK: s1 %li, s2 %li", li, ri);
        throw std::runtime_error("END OF TRACK");
    }
#endif

    if (li <= BLACK && ri <= BLACK)
        return 0;
    else
        return _calcHorizontalPosition(li, ri, lo, ro);
}

void SensorManager::calibrate()
{
#ifdef PRE_CALIBRATION
    /*
    calibration = {{85, 1844},
                   {105, 1753},
                   {60, 1709},
                   {120, 1801},

                   {58, 1771},
                   {90, 1858},
                   {79, 1734},
                   {12, 384},

                   {82, 1838},
                   {80, 1843},
                   {80, 1878},
                   {63, 1816}};
                   */

    calibration = {{94, 1829},
                   {105, 1854},
                   {80, 1519},
                   {120, 1817},

                   {58, 1742},
                   {90, 1852},
                   {79, 1862},
                   {12, 559},

                   {82, 1674},
                   {80, 1735},
                   {80, 1809},
                   {63, 1773}};

    /*
     * FLOw: 1834, FLOb: 88, FLIw: 1856, FLIb: 101, FRIw: 1519, FRIb: 70, FROw: 1863, FROb: 123
CLOw: 1300, CLOb: 66, CLIw: 1717, CLIb: 85, CRIw: 1504, CRIb: 81, CROw: 329, CROb: 10
BLOw: 1674, BLOb: 76, BLIw: 1735, BLIb: 79, BRIw: 1809, BRIb: 83, BROw: 1773, BROb: 78
     */

    /*
     * FLOw: 1836, FLOb: 84, FLIw: 1856, FLIb: 101, FRIw: 1589, FRIb: 64, FROw: 1412, FROb: 122
CLOw: 1456, CLOb: 64, CLIw: 1825, CLIb: 87, CRIw: 1563, CRIb: 83, CROw: 326, CROb: 10
BLOw: 1783, BLOb: 76, BLIw: 1804, BLIb: 78, BRIw: 1838, BRIb: 84, BROw: 1797, BROb: 78
     */


    /*
    calibration = {{93, 1852},
                   {105, 1675},
                   {62, 1730},
                   {108, 1049},

                   {58, 1772},
                   {97, 1848},
                   {74, 1829},
                   {8, 356},

                   {72, 1845},
                   {76, 1838},
                   {80, 1845},
                   {70, 1838}};
    */
#endif

#ifndef PRE_CALIBRATION
    motor_manager->turn(90, LEFT);
    motor_manager->creepDistance(1, FORWARD);

    int32_t FLIb = readRawSensor(SENSOR_FLI);
    int32_t FRIb = readRawSensor(SENSOR_FRI);
    int32_t FLOb = readRawSensor(SENSOR_FLO);
    int32_t FROb = readRawSensor(SENSOR_FRO);

    motor_manager->creepDistance(6, BACKWARD);

    int32_t FLIw = readRawSensor(SENSOR_FLI);
    int32_t FRIw = readRawSensor(SENSOR_FRI);
    int32_t FLOw = readRawSensor(SENSOR_FLO);
    int32_t FROw = readRawSensor(SENSOR_FRO);

    int32_t CLIb = readRawSensor(SENSOR_CLI);
    int32_t CRIb = readRawSensor(SENSOR_CRI);
    int32_t CLOb = readRawSensor(SENSOR_CLO);
    int32_t CROb = readRawSensor(SENSOR_CRO);

    int32_t BLIw = readRawSensor(SENSOR_BLI);
    int32_t BRIw = readRawSensor(SENSOR_BRI);
    int32_t BLOw = readRawSensor(SENSOR_BLO);
    int32_t BROw = readRawSensor(SENSOR_BRO);

    motor_manager->creepDistance(5, BACKWARD);

    int32_t BLIb = readRawSensor(SENSOR_BLI);
    int32_t BRIb = readRawSensor(SENSOR_BRI);
    int32_t BLOb = readRawSensor(SENSOR_BLO);
    int32_t BROb = readRawSensor(SENSOR_BRO);

    int32_t CLIw = readRawSensor(SENSOR_CLI);
    int32_t CRIw = readRawSensor(SENSOR_CRI);
    int32_t CLOw = readRawSensor(SENSOR_CLO);
    int32_t CROw = readRawSensor(SENSOR_CRO);

    motor_manager->creepDistance(10, FORWARD);
    motor_manager->turn(90, RIGHT);

    calibration.emplace_back(FLOb, FLOw);
    calibration.emplace_back(FLIb, FLIw);
    calibration.emplace_back(FRIb, FRIw);
    calibration.emplace_back(FROb, FROw);

    calibration.emplace_back(CLOb, CLOw);
    calibration.emplace_back(CLIb, CLIw);
    calibration.emplace_back(CRIb, CRIw);
    calibration.emplace_back(CROb, CROw);

    calibration.emplace_back(BLOb, BLOw);
    calibration.emplace_back(BLIb, BLIw);
    calibration.emplace_back(BRIb, BRIw);
    calibration.emplace_back(BROb, BROw);

    printf("FLOw: %li, FLOb: %li, FLIw: %li, FLIb: %li, FRIw: %li, FRIb: %li, FROw: %li, FROb: %li\n\r", FLOw, FLOb, FLIw, FLIb, FRIw, FRIb, FROw, FROb);
    printf("CLOw: %li, CLOb: %li, CLIw: %li, CLIb: %li, CRIw: %li, CRIb: %li, CROw: %li, CROb: %li\n\r", CLOw, CLOb, CLIw, CLIb, CRIw, CRIb, CROw, CROb);
    printf("BLOw: %li, BLOb: %li, BLIw: %li, BLIb: %li, BRIw: %li, BRIb: %li, BROw: %li, BROb: %li\n\r", BLOw, BLOb, BLIw, BLIb, BRIw, BRIb, BROw, BROb);

    while (true)
        sleep_ms(100);
#endif

    calibrated = true;
}

int32_t SensorManager::_mapDistance(int32_t s, std::pair<int32_t, int32_t> calib)
{
    double cb = s-calib.first;
    double wb = calib.second-calib.first;
    double div = cb/wb*1000.0;
    auto ret = (int32_t)div;

    return ret;
}

bool SensorManager::isCalibrated()
{
    return calibrated;
}

int32_t SensorManager::getFullHorizontalPosition(SensorRow row)
{
    int32_t li, ri, lo, ro;

    switch (row)
    {
        case SENSOR_ROW_FRONT:
            li = readSensor(SENSOR_FLI);
            ri = readSensor(SENSOR_FRI);
            lo = readSensor(SENSOR_FLO);
            ro = readSensor(SENSOR_FRO);
            break;
        case SENSOR_ROW_CENTER:
            li = readSensor(SENSOR_CLI);
            ri = readSensor(SENSOR_CRI);
            lo = readSensor(SENSOR_CLO);
            ro = readSensor(SENSOR_CRO);
            break;
        case SENSOR_ROW_BACK:
            li = readSensor(SENSOR_BLI);
            ri = readSensor(SENSOR_BRI);
            lo = readSensor(SENSOR_BLO);
            ro = readSensor(SENSOR_BRO);
            break;
    }

#ifdef ENABLE_END_OF_TRACK
    if (row == SENSOR_ROW_BACK && li > WHITE && ri > WHITE)
    {
        motor_manager->setDirection(STOP);
        printf("END OF TRACK: s1 %li, s2 %li", li, ri);
        throw std::runtime_error("END OF TRACK");
    }
#endif

    if (li <= BLACK && ri <= BLACK)
        return 0;
    else
        return ri + ro - li - lo;
}
