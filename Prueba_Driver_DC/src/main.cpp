#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "uarts.h"
#include "tareas.h"
#include "adcs.h"
#include "gpio_init.h"
#include "pwm_init.h"

void setup()
{
  init_gpio();                                                                                    // inicializo salidas y entradas
  initUART0();                                                                                    // inicializo uart0
  init_adc();                                                                                     // inicializo ADCs
  pwm_init();                                                                                     // inicializo salidas PWM
  xTaskCreatePinnedToCore(TareaEntradaDatos, "Tarea_para_entrada1", 1024 * 5, NULL, 12, NULL, 1); // creo tarea1
}

void loop()
{
}