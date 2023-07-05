#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"

#define OUT1 GPIO_NUM_25
#define OUT2 GPIO_NUM_33
#define OUT3 GPIO_NUM_27
#define OUT4 GPIO_NUM_26
#define LED2 GPIO_NUM_2
#define B1 GPIO_NUM_0

esp_err_t init_gpio(void);
int toggle(char x);