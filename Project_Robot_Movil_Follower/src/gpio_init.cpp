#include "gpio_init.h"

//void isr_handler(void *arg);

void gpio_init()
{

    gpio_reset_pin(OUT1);
    gpio_set_direction(OUT1, GPIO_MODE_OUTPUT);
    gpio_reset_pin(OUT2);
    gpio_set_direction(OUT2, GPIO_MODE_OUTPUT);
    gpio_reset_pin(OUT3);
    gpio_set_direction(OUT3, GPIO_MODE_OUTPUT);
    gpio_reset_pin(OUT4);
    gpio_set_direction(OUT4, GPIO_MODE_OUTPUT);

    gpio_set_level(OUT1,1);
    gpio_set_level(OUT3,1);
    gpio_set_level(OUT2,0);
    gpio_set_level(OUT4,0);

}