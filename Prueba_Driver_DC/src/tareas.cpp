#include "adcs.h"
#include "gpio_init.h"
#include "pwm_init.h"
#include "modbusTCPIP.h"
#include "adcs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern UINT16_VAL MBHoldingRegister[maxHoldingRegister];
extern UINT16_VAL MBInputRegister[maxInputRegister];
extern UINT16_VAL MBCoils;
extern UINT16_VAL MBDiscreteInputs;

//bool x1 = 0, x2 = 0;
uint16_t DutyM1, DutyM2;
// extern int rssi;

void TareaEntradaDatos(void *Parametro)
{
    uint32_t promedio1 = 0;
    uint32_t promedio2 = 0;
    // uint32_t promedio3 = 0;
    // int8_t promedio4 = 0;

    uint32_t adc_value1 = adc1_get_raw(CH1);
    uint32_t adc_value2 = adc1_get_raw(CH2);
    // uint32_t adc_value3 = adc1_get_raw(CH3);

    while (1)
    {
        // Iteración valores ADCs
        for (int j = 0; j < NumeroMuestras; j++)
        {

            promedio1 += adc_value1 = adc1_get_raw(CH1);
            promedio2 += adc_value2 = adc1_get_raw(CH2);
            // promedio3 += adc_value3 = adc1_get_raw(CH3);
        }
        // Calculo Promedio de 100 muestras
        promedio1 /= NumeroMuestras;
        promedio2 /= NumeroMuestras;
        // promedio3 /= NumeroMuestras;

        // Establezco límite superior de entrada
        if (promedio1 > 1024)
            promedio1 = 1024;
        if (promedio2 > 1024)
            promedio2 = 1024;

        // Escribo en el Imput Register
        MBInputRegister[0].Val = promedio1;
        MBInputRegister[1].Val = promedio2;

        DutyM1 = MBHoldingRegister[0].Val;
        DutyM2 = MBHoldingRegister[1].Val;
        set_pwm();

/*
        // Compruebo activación de botones
        if (int b1 = gpio_get_level(BT1) == 0)
        {

            x1 = toggle(x1);

            MBCoils.bits.b0 = !x1;

            MBCoils.bits.b1 = x1;

            vTaskDelay(pdMS_TO_TICKS(200));
        }

        if (int b2 = gpio_get_level(BT2) == 0)
        {

            x2 = toggle(x2);

            MBCoils.bits.b2 = !x2;

            MBCoils.bits.b3 = x2;

            vTaskDelay(pdMS_TO_TICKS(200));
        }
*/
        // MBInputRegister[2].Val = promedio3;
        // MBInputRegister[3].Val = rssi;

        // printf("valor adc3 crudo: %d\n", adc_value3);
        // printf("valor adc3 suave: %d\n", promedio3);

        gpio_set_level(C0, MBCoils.bits.b0);
        gpio_set_level(C1, MBCoils.bits.b1);
        gpio_set_level(C2, MBCoils.bits.b2);
        gpio_set_level(C3, MBCoils.bits.b3);

        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}

void set_pwm()
{

    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, DutyM2);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
    vTaskDelay(20 / portTICK_PERIOD_MS);

    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, DutyM1);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
    vTaskDelay(20 / portTICK_PERIOD_MS);
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
