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
                   {80, 1600},
                   {120, 1817},

                   {58, 1742},
                   {90, 1852},
                   {79, 1862},
                   {12, 559},

                   {82, 1838},
                   {80, 1843},
                   {80, 1878},
                   {63, 1816}};

    /*
     * FLOw: 1833, FLOb: 82, FLIw: 1854, FLIb: 96, FRIw: 1608, FRIb: 75, FROw: 1436, FROb: 144
     * CLOw: 1742, CLOb: 75, CLIw: 1852, CLIb: 86, CRIw: 1862, CRIb: 123, CROw: 559, CROb: 11
     * BLOw: 1853, BLOb: 105, BLIw: 1843, BLIb: 94, BRIw: 1855, BRIb: 122, BROw: 1845, BROb: 96
     */

    /*
     * FLOw: 1848, FLOb: 85, FLIw: 1854, FLIb: 105, FRIw: 1709, FRIb: 80, FROw: 901, FROb: 160
     * CLOw: 1571, CLOb: 77, CLIw: 1858, CLIb: 90, CRIw: 1734, CRIb: 79, CROw: 384, CROb: 11
     * BLOw: 1838, BLOb: 82, BLIw: 1643, BLIb: 80, BRIw: 1678, BRIb: 66, BROw: 1716, BROb: 63
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
    /*
     * FLOw: 1844, FLOb: 118, FLIw: 1853, FLIb: 139, FRIw: 1832, FRIb: 76, FROw: 899, FROb: 120
     * CLOw: 1341, CLOb: 85, CLIw: 1598, CLIb: 118, CRIw: 1653, CRIb: 89, CROw: 320, CROb: 9
     * BLOw: 1802, BLOb: 110, BLIw: 1758, BLIb: 92, BRIw: 1719, BRIb: 95, BROw: 1606, BROb: 81
     */

    /*
     * FLOw: 1831, FLOb: 102, FLIw: 1857, FLIb: 121, FRIw: 1382, FRIb: 90, FROw: 1863, FROb: 147
     * CLOw: 1589, CLOb: 69, CLIw: 1851, CLIb: 104, CRIw: 1837, CRIb: 102, CROw: 375, CROb: 13
     * BLOw: 1841, BLOb: 85, BLIw: 1806, BLIb: 92, BRIw: 1851, BRIb: 102, BROw: 1786, BROb: 85
     */

    /*
     * FLOw: 1739, FLOb: 84, FLIw: 1853, FLIb: 96, FRIw: 1489, FRIb: 81, FROw: 1862, FROb: 140
     * CLOw: 1312, CLOb: 66, CLIw: 1633, CLIb: 103, CRIw: 1815, CRIb: 93, CROw: 321, CROb: 12
     * BLOw: 1859, BLOb: 80, BLIw: 1846, BLIb: 86, BRIw: 1799, BRIb: 100, BROw: 1695, BROb: 81
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
