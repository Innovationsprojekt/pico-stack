/**
 * @file adc_wrapper.h
 * @author Noa Sendlhofer
 */

#ifndef PICO_MOTORS_ADC_WRAPPER_H_
#define PICO_MOTORS_ADC_WRAPPER_H_

#include "pico/stdlib.h"
#include "ads1x15/ads1x15.hpp"

class ADCWrapper
{
public:
    ADCWrapper(ADSXAddressI2C_e i2c_addr, i2c_inst_t* i2c_type, uint16_t CLK_speed, uint8_t SDA_pin, uint8_t SCL_pin);
    int16_t readADC(ADSX_AINX_e channel);
    float readVoltage(ADSX_AINX_e channel);

private:
    PICO_ADS1015 ads;
};


#endif //PICO_MOTORS_ADC_WRAPPER_H_
