/**
 * @file sensor.h
 * @author Noa Sendlhofer
 */

#ifndef PICO_MOTORS_SENSOR_H_
#define PICO_MOTORS_SENSOR_H_

#include <memory>
#include "pico/stdlib.h"
#include "adc_wrapper.h"

class Sensor
{
public:
    Sensor(std::shared_ptr<ADCWrapper> adc, ADSX_AINX_e channel);
    int16_t getSensorValue();

private:
    std::shared_ptr<ADCWrapper> adc;
    ADSX_AINX_e channel;
};


#endif //PICO_MOTORS_SENSOR_H_