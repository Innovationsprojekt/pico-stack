/**
 * @file sensor.cpp
 * @author Noa Sendlhofer
 */

#include "sensor.h"
#include <utility>

Sensor::Sensor(std::shared_ptr<ADCWrapper> adc
               , ADSX_AINX_e channel)
               : adc(std::move(adc))
               , channel(channel)
{
}

int16_t Sensor::getSensorValue()
{
    return adc->readADC(channel);
}
