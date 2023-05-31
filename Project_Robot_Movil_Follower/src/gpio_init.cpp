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
    
    gpio_reset_pin(IN1);
    gpio_set_direction(IN1, GPIO_MODE_INPUT);
    gpio_reset_pin(IN2);
    gpio_set_direction(IN2, GPIO_MODE_INPUT);
/*
    gpio_config_t gpioConfig1;
    gpioConfig1.pin_bit_mask = (1ULL << IN1);
    gpioConfig1.mode = GPIO_MODE_INPUT;
    gpioConfig1.pull_up_en = GPIO_PULLUP_DISABLE;
    gpioConfig1.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpioConfig1.intr_type = GPIO_INTR_NEGEDGE;

    gpio_config(&gpioConfig1);

    gpio_config_t gpioConfig2;
    gpioConfig2.pin_bit_mask = (1ULL << IN2);
    gpioConfig2.mode = GPIO_MODE_INPUT;
    gpioConfig2.pull_up_en = GPIO_PULLUP_DISABLE;
    gpioConfig2.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpioConfig2.intr_type = GPIO_INTR_NEGEDGE;

    gpio_config(&gpioConfig2);

    gpio_install_isr_service(0);

    gpio_isr_handler_add(IN1, isr_handler, NULL);
    gpio_isr_handler_add(IN2, isr_handler, NULL);
*/

    gpio_set_level(OUT1,1);
    gpio_set_level(OUT3,1);
    gpio_set_level(OUT2,0);
    gpio_set_level(OUT4,0);

}