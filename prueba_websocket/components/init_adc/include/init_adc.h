#include "driver/adc.h"
#include "esp_log.h"
#include "esp_err.h"

#define CH1 ADC1_CHANNEL_5

#define NumeroMuestras 65

typedef struct
{
    uint16_t Sensor1;
} Data_adc;

esp_err_t init_adc(void);