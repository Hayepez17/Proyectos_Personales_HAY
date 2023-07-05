#include <stdio.h>
#include "init_gpio.h"

static char *tag = "GPIO";

esp_err_t init_gpio(void)
{
    /*
    gpio_reset_pin(OUT1);
    gpio_set_direction(OUT1, GPIO_MODE_OUTPUT);
    gpio_reset_pin(OUT2);
    gpio_set_direction(OUT2, GPIO_MODE_OUTPUT);
    gpio_reset_pin(OUT3);
    gpio_set_direction(OUT3, GPIO_MODE_OUTPUT);
    gpio_reset_pin(OUT4);
    gpio_set_direction(OUT4, GPIO_MODE_OUTPUT);
 */
    gpio_reset_pin(LED2);
    gpio_set_direction(LED2, GPIO_MODE_OUTPUT);

    gpio_reset_pin(B1);
    gpio_set_direction(B1, GPIO_MODE_INPUT);

    ESP_LOGI(tag, "Init gpio completed");

    return ESP_OK;
}

int toggle(char x)
{
    if (x)
    {
        x = 0;
        return x;
    }
    x = 1;
    return x;
}