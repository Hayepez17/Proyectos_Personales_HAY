#include "driver/adc.h"
#include "esp_log.h"
#include "esp_err.h"

#define CH1 ADC1_CHANNEL_6
#define CH2 ADC1_CHANNEL_7

#define NumeroMuestras 60

//esp_err_t init_adc(void);