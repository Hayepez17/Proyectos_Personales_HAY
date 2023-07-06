#include <stdio.h>
#include "init_pwm.h"


static char *tag = "Diver DC";

esp_err_t init_pwm(void)
{

    ledc_timer_config_t ConfigTimer1PWM;
    ConfigTimer1PWM.speed_mode = LEDC_HIGH_SPEED_MODE;
    ConfigTimer1PWM.duty_resolution = LEDC_TIMER_9_BIT;
    ConfigTimer1PWM.timer_num = LEDC_TIMER_1;
    ConfigTimer1PWM.freq_hz = 5000;
    ConfigTimer1PWM.clk_cfg = LEDC_USE_APB_CLK;

    ledc_timer_config(&ConfigTimer1PWM);

    ledc_timer_config_t ConfigTimer2PWM;
    ConfigTimer2PWM.speed_mode = LEDC_HIGH_SPEED_MODE;
    ConfigTimer2PWM.duty_resolution = LEDC_TIMER_9_BIT;
    ConfigTimer2PWM.timer_num = LEDC_TIMER_2;
    ConfigTimer2PWM.freq_hz = 5000;
    ConfigTimer2PWM.clk_cfg = LEDC_USE_APB_CLK;

    ledc_timer_config(&ConfigTimer2PWM);

    ledc_channel_config_t ConfigPWM1;
    ConfigPWM1.gpio_num = M1;
    ConfigPWM1.speed_mode = LEDC_HIGH_SPEED_MODE;
    ConfigPWM1.channel = CM1;
    ConfigPWM1.intr_type = LEDC_INTR_DISABLE;
    ConfigPWM1.timer_sel = LEDC_TIMER_1;
    ConfigPWM1.duty = 0;

    ledc_channel_config_t ConfigPWM2;
    ConfigPWM2.gpio_num = M2;
    ConfigPWM2.speed_mode = LEDC_HIGH_SPEED_MODE;
    ConfigPWM2.channel = CM2;
    ConfigPWM2.intr_type = LEDC_INTR_DISABLE;
    ConfigPWM2.timer_sel = LEDC_TIMER_2;
    ConfigPWM2.duty = 0;

    ledc_channel_config(&ConfigPWM1);
    ledc_channel_config(&ConfigPWM2);

    ESP_LOGI(tag, "PWM init completed");

    return ESP_OK;
}

void set_pwm(uint16_t duty1, uint16_t duty2, int on)
{
    
if (on)
{
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, CM1, duty1);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, CM1);
    vTaskDelay(pdMS_TO_TICKS(10));

    ledc_set_duty(LEDC_HIGH_SPEED_MODE, CM2, duty2);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, CM2);
    vTaskDelay(pdMS_TO_TICKS(10));
}

    else
    {
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, CM1, 0);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, CM1);
    vTaskDelay(pdMS_TO_TICKS(10));

    ledc_set_duty(LEDC_HIGH_SPEED_MODE, CM2, 0);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, CM2);
    vTaskDelay(pdMS_TO_TICKS(10));/* code */
    }
    
}
