#include "gpio_init.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void init_gpio()
{

    gpio_reset_pin(C0);
    gpio_set_direction(C0, GPIO_MODE_OUTPUT);
    gpio_reset_pin(C1);
    gpio_set_direction(C1, GPIO_MODE_OUTPUT);
    gpio_reset_pin(C2);
    gpio_set_direction(C2, GPIO_MODE_OUTPUT);
    gpio_reset_pin(C3);
    gpio_set_direction(C3, GPIO_MODE_OUTPUT);

    gpio_reset_pin(BT1);
    gpio_set_direction(BT1, GPIO_MODE_INPUT);
    gpio_reset_pin(BT2);
    gpio_set_direction(BT2, GPIO_MODE_INPUT);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    
}