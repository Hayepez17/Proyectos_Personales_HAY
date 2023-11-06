#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"

#define LED2 GPIO_NUM_2


#define BT GPIO_NUM_0

esp_err_t init_gpio(void);