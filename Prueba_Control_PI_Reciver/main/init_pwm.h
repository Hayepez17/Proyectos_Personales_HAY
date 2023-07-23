#include "esp_log.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define M1 GPIO_NUM_22
#define M2 GPIO_NUM_4

#define CM1 LEDC_CHANNEL_0
#define CM2 LEDC_CHANNEL_1

#define portTICK_PERIOD_US              ( ( TickType_t ) 1000000 / configTICK_RATE_HZ )

enum DIRECCTIONS
    {
        STOP,
        FORWARD,
        LEFT,
        RIGHT,
        BACK
    };

esp_err_t init_pwm(void);
void set_pwm(uint16_t duty1, uint16_t duty2);