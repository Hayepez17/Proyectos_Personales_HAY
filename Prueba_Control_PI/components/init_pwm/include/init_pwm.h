#include "esp_log.h"
#include "driver/ledc.h"
#include "esp_err.h"

#define M1 GPIO_NUM_32
#define M2 GPIO_NUM_14

#define CM1 LEDC_CHANNEL_1
#define CM2 LEDC_CHANNEL_2

//esp_err_t init_pwm(void);
//void set_pwm(char chanel, uint16_t duty);