#include <Arduino.h>
#include "tareas.h"
#include "ultrasonic.h"
#include "def20.h"
#include "uarts.h"
#include "freertos/FreeRTOS.h"

static QueueHandle_t uart0_queue;
#define tamBUFFER 1024

// TaskHandle_t xHandle1;

int boton;

void EnvioDatos(void *Parametro)
{
  ultrasonic_sensor_t sensorU = {
      .trigger_pin = TRIGGER_GPIO,
      .echo_pin = ECHO_GPIO};

  float_VAL distance;
  uint8_t dataTX[4];
 
  ultrasonic_init(&sensorU);

  while (1)
  {
    if (boton)
    {
      esp_err_t res = ultrasonic_measure(&sensorU, MAX_DISTANCE_CM, &distance.Val);
      if (res != ESP_OK)
      {
        printf("Error %d: ", res);
        switch (res)
        {
        case ESP_ERR_ULTRASONIC_PING:
          printf("Cannot ping (device is in invalid state)\n");
          break;
        case ESP_ERR_ULTRASONIC_PING_TIMEOUT:
          printf("Ping timeout (no device found)\n");
          break;
        case ESP_ERR_ULTRASONIC_ECHO_TIMEOUT:
          printf("Echo timeout (i.e. distance too big)\n");
          distance.Val = MAX_DISTANCE_CM;
          break;
        default:
          printf("%s\n", esp_err_to_name(res));
        }
      }

      dataTX[0]=distance.byte.Byte0;
      dataTX[1]=distance.byte.Byte1;
      dataTX[2]=distance.byte.Byte2;
      dataTX[3]=distance.byte.Byte3;

           uart_write_bytes(UART_NUM_0, (const char *)dataTX, sizeof(dataTX));

          //printf("%u cm \n", distance.byte.LB);

      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    elseP
      vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void TareaEventosUART0(void *Parametro)
{
  uart_event_t evento;

  uint8_t *datoRX = (uint8_t *)malloc(tamBUFFER);

  gpio_reset_pin(GPIO_NUM_2);
  gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);

  for (;;)
  {
    if (xQueueReceive(uart0_queue, (void *)&evento, (TickType_t)portMAX_DELAY))
    {
      bzero(datoRX, tamBUFFER);
      if (evento.type == UART_DATA)
      {
        uart_read_bytes(UART_NUM_0, datoRX, evento.size, portMAX_DELAY);
        switch (datoRX[0])
        {
        case 1:
          gpio_set_level(GPIO_NUM_2, 1);
          boton = 1;
          break;

        case 0:
          gpio_set_level(GPIO_NUM_2, 0);
          boton = 0;
          break;

        default:
          break;
        }
      }
    }
  }
}

//**************************************
//************* Init UARTs *************
//**************************************

// Función Para iniciar el UART0
void initUART0()
{
  uart_config_t configUART0 = {
      .baud_rate = 115200,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_APB,
  };
  // configUART0.baud_rate=9600;

  uart_param_config(UART_NUM_0, &configUART0);
  uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_driver_install(UART_NUM_0, tamBUFFER * 2, tamBUFFER * 2, 20, &uart0_queue, 0);

  xTaskCreatePinnedToCore(TareaEventosUART0, "Tarea_para_UART0", 1024 * 5, NULL, 12, NULL, 1);
}

void setup()
{
  boton = 0;
  initUART0();                                                                                 // inicializo uart0
  xTaskCreatePinnedToCore(EnvioDatos, "Tarea_para_envio_serial", 1024 * 5, NULL, 12, NULL, 0); // creo tarea
}

void loop()
{
}