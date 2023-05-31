#include "adc_init.h"

void adc_init()
{

    adc1_config_width(ADC_WIDTH_BIT_10);
    adc1_config_channel_atten(CH1, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(CH2, ADC_ATTEN_DB_11);
}