/**
 * @file adc_wrapper.cpp
 * @author Noa Sendlhofer
 */

#include <stdexcept>
#include "adc_wrapper.h"

ADCWrapper::ADCWrapper(ADSXAddressI2C_e i2c_addr,
                       i2c_inst_t* i2c_type,
                       uint16_t CLK_speed,
                       uint8_t SDA_pin,
                       uint8_t SCL_pin)
{
    ads.setGain(ADSXGain_ONE);

    if (!ads.beginADSX(i2c_addr, i2c_type, CLK_speed, SDA_pin,SCL_pin))
        throw std::runtime_error("Initializing ADC failed");
}

int16_t ADCWrapper::readADC(ADSX_AINX_e channel)
{
    return ads.readADC_SingleEnded(channel);
}

float ADCWrapper::readVoltage(ADSX_AINX_e channel)
{
    int16_t res = ads.readADC_SingleEnded(channel);
    return ads.computeVolts(res);
}
