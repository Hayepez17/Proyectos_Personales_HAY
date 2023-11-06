#include <stdio.h>
#include "init_gpio.h"

static char *tag = "GPIO";

esp_err_t init_gpio(void)
{

    gpio_reset_pin(LED2);
    gpio_set_direction(LED2, GPIO_MODE_OUTPUT);

    gpio_reset_pin(BT);
    gpio_set_direction(BT, GPIO_MODE_INPUT);

    gpio_set_level(LED2, 0);

    ESP_LOGI(tag, "Init gpio completed");

    return ESP_OK;
}
