#include "driver/adc.h"
#include "esp_adc_cal.h"

#define CH1 ADC1_CHANNEL_4
#define CH2 ADC1_CHANNEL_5
//#define CH3 ADC1_CHANNEL_6

#define NumeroMuestras 60

void init_adc();
