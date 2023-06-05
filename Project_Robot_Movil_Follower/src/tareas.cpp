#include "adc_init.h"
#include "gpio_init.h"
#include "pwm_init.h"
#include "tareas.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


bool x1 = 0, x2 = 0;

uint16_t DutyM1, DutyM2;

void Tarea1(void *Parametro)
{
    uint32_t promedio1 = 0;
    uint32_t promedio2 = 0;

    uint32_t adc_value1 = adc1_get_raw(CH1);
    uint32_t adc_value2 = adc1_get_raw(CH2);

    while (1)
    {

        for (int j = 0; j < NumeroMuestras; j++)
        {

            promedio1 += adc_value1 = adc1_get_raw(CH1);
            promedio2 += adc_value2 = adc1_get_raw(CH2);

            //vTaskDelay(20);
        }

        promedio1 /= NumeroMuestras;
        promedio2 /= NumeroMuestras;

        if (promedio1 > 1024)
            promedio1 = 1024;
        if (promedio2 > 1024)
            promedio2 = 1024;

        DutyM1 = promedio1;
        DutyM2 = promedio2;
        set_pwm();

        printf("adc1: %d \n", DutyM1);
        printf("adc2: %d \n", DutyM2);

        // Compruebo activación de botones
        if (int b1 = gpio_get_level(IN1) == 0)
        {

            x1 = toggle(x1);

            gpio_set_level(OUT1, !x1);
            gpio_set_level(OUT2, x1);
            vTaskDelay(pdMS_TO_TICKS(200));
        }

        if (int b2 = gpio_get_level(IN2) == 0)
        {

            x2 = toggle(x2);

            gpio_set_level(OUT3, !x2);
            gpio_set_level(OUT4, x2);

            vTaskDelay(pdMS_TO_TICKS(200));
        }

    }
}

/*void Tarea2(void *Parametro)
{
    while (1)
    {
        if (int b1 = gpio_get_level(IN1) == 0)
        {

            x1 = toggle(x1);

            gpio_set_level(OUT1, !x1);
            gpio_set_level(OUT2, x1);

            vTaskDelay(pdMS_TO_TICKS(200));
        }

        if (int b2 = gpio_get_level(IN2) == 0)
        {


            x2 = toggle(x2);

            gpio_set_level(OUT3, !x2);
            gpio_set_level(OUT4, x2);

            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }
}
*/
void set_pwm()
{

    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, DutyM1);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);

    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, DutyM2);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);

    vTaskDelay(pdMS_TO_TICKS(20));
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