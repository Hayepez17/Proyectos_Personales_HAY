
#include "adc_init.h"
#include "gpio_init.h"
#include "pwm_init.h"
#include "tareas.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


void setup() {
gpio_init();
adc_init();
pwm_init();
xTaskCreatePinnedToCore(Tarea1, "Tarea_para_DriverDC", 1024 * 5, NULL, 1, NULL, 1); // creo tarea1
//xTaskCreatePinnedToCore(Tarea2, "Tarea_para_botones", 1024 * 2, NULL, 2, NULL, 0); // creo tarea1

}

void loop() 
{
}