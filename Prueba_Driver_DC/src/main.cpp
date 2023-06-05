#include "tareas.h"
#include "adcs.h"
#include "gpio_init.h"
#include "pwm_init.h"
#include "modbusTCPIP.h"

void setup()
{
  init_gpio(); // inicializo salidas y entradas                                                                                   // inicializo uart0
  init_adc();  // inicializo ADCs
  setupWiFiServer();
  pwm_init(); // inicializo salidas PWM
  xTaskCreatePinnedToCore(TareaEntradaDatos, "Tarea_para_entrada1", 1024 * 10, NULL, 12, NULL, 1); // creo tarea1
}

void loop()
{
}