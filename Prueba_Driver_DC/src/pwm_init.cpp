#include "pwm_init.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


void pwm_init(){

    ledc_timer_config_t ConfigTimerPWM;
    ConfigTimerPWM.speed_mode = LEDC_HIGH_SPEED_MODE;
    ConfigTimerPWM.duty_resolution = LEDC_TIMER_10_BIT;
    ConfigTimerPWM.timer_num = LEDC_TIMER_1;
    ConfigTimerPWM.freq_hz = 5000;

    ledc_timer_config(&ConfigTimerPWM);

    ledc_channel_config_t ConfigPWM1;
    ConfigPWM1.gpio_num = M1;
    ConfigPWM1.speed_mode = LEDC_HIGH_SPEED_MODE;
    ConfigPWM1.channel = LEDC_CHANNEL_0;
    ConfigPWM1.intr_type = LEDC_INTR_DISABLE;
    ConfigPWM1.timer_sel = LEDC_TIMER_1;
    ConfigPWM1.duty = 0;

    ledc_channel_config_t ConfigPWM2;
    ConfigPWM2.gpio_num = M2;
    ConfigPWM2.speed_mode = LEDC_HIGH_SPEED_MODE;
    ConfigPWM2.channel = LEDC_CHANNEL_1;
    ConfigPWM2.intr_type = LEDC_INTR_DISABLE;
    ConfigPWM2.timer_sel = LEDC_TIMER_1;
    ConfigPWM2.duty = 0;
    
    ledc_channel_config(&ConfigPWM1);
    ledc_channel_config(&ConfigPWM2);

}