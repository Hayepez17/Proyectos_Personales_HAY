#include "driver/gpio.h"

#define OUT1 GPIO_NUM_22
#define OUT2 GPIO_NUM_23
#define OUT3 GPIO_NUM_18
#define OUT4 GPIO_NUM_19


void gpio_init(void);
int toggle(char x);