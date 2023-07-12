#include <stdio.h>
#include <string.h>
#include <assert.h>

#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_now.h"
#include "esp_mac.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "init_gpio.h"
#include "init_pwm.h"
#include "init_adc.h"
#include "tareas.h"
#include "ControlPID.h"

#define ESP_CHANNEL 1

// Queue parameters
#define QUEUE_LENGTH 1
#define ITEM_SIZE sizeof(Datos)

QueueHandle_t xQueue;

// buffer esp now send

char BUFF[32];

// Mac addres peer esp now

//static uint8_t peer_mac[ESP_NOW_ETH_ALEN] = {0xc8, 0xf0, 0x9e, 0xf1, 0x6b, 0x10};
 static uint8_t peer_mac[ESP_NOW_ETH_ALEN] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

// etiquetas

static char *TAG = "esp_now_resp";

// Estructuras y variables globales

typedef struct
{
    bool ON;
    uint16_t Vel;
    float KP;
    float KI;
    bool Sentido1;
    bool Sentido2;
    int offset;
} Datos;

Datos CtrlData;

uint16_t dutym1;
uint16_t dutym2;
//******************PROTOTIPO DE FUNCIONES********************

static esp_err_t esp_now_send_data(const uint8_t *peer_addr, const uint8_t *data, size_t len);

//************FUNCIÓN DE INICIALIZACIÓN DE WIFI********************

static esp_err_t init_wifi(void)
{

    wifi_init_config_t wifi_inti_config = WIFI_INIT_CONFIG_DEFAULT();

    esp_netif_init();
    esp_event_loop_create_default();
    nvs_flash_init();
    esp_wifi_init(&wifi_inti_config);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_storage(WIFI_STORAGE_FLASH);
    esp_wifi_start();

    ESP_LOGI(TAG, "wifi init completed");
    return ESP_OK;
}

//*****************FUNCIONES DE CALLBACK ESPNOW****************************

void recv_cb(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len)
{

    ESP_LOGI(TAG, "Data recived " MACSTR " %s", MAC2STR(esp_now_info->src_addr), data);

    CtrlData.ON = atoi(strtok((char *)data, "|"));  // "ON|Vel|KP|KI|S1|S2|offset" obtiene el primer valor de la cadena y guarda en variable
    CtrlData.Vel = atoi(strtok(NULL, "|"));         // "ON|Vel|KP|KI|S1|S2|offset" obtiene el segundo valor de la cadena y guarda en variable
    CtrlData.KP = atoi(strtok(NULL, "|")) / 100.0f; // "ON|Vel|KP|KI|S1|S2|offset" obtiene el tercer valor de la cadena y guarda en variable
    CtrlData.KI = atoi(strtok(NULL, "|")) / 100.0f; // "ON|Vel|KP|KI|S1|S2|offset" obtiene el cuarto valor de la cadena y guarda en variable
    CtrlData.Sentido1 = atoi(strtok(NULL, "|"));    // "ON|Vel|KP|KI|S1|S2|offset" obtiene el quinto valor de la cadena y guarda en variable
    CtrlData.Sentido2 = atoi(strtok(NULL, "|"));    // "ON|Vel|KP|KI|S1|S2|offset" obtiene el sexto valor de la cadena y guarda en variable
    CtrlData.offset = atoi(strtok(NULL, "|"));      // "ON|Vel|KP|KI|S1|S2|offset" obtiene el septimo valor de la cadena y guarda en variable

    snprintf(BUFF, sizeof(BUFF), "%u|%u", dutym1, dutym2);  // Arreglo cadena de caracteres en forma "dutym1|dutym2"
    esp_now_send_data(peer_mac, (const uint8_t *)BUFF, 32); // Envía cadena en forma "dutym1|dutym2"
}

void send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    if (status == ESP_NOW_SEND_SUCCESS)
    {
        ESP_LOGI(TAG, "ESP_NOW_SEND_SUCCESS");
    }
    else
    {
        ESP_LOGW(TAG, "ESP_NOW_SEND_FAIL");
    }
}

//************FUNCIÓN DE INICIALIZACIÓN DE ESPNOW**********************

static esp_err_t init_esp_now(void)
{
    esp_now_init();
    esp_now_register_recv_cb(recv_cb);
    esp_now_register_send_cb(send_cb);

    ESP_LOGI(TAG, "esp now init completed");
    return ESP_OK;
}

//******************FUNCIÓN DE REGISTRO ESPNOW*************************

static esp_err_t register_peer(uint8_t *peer_addr)
{
    esp_now_peer_info_t esp_now_peer_info = {};
    memcpy(esp_now_peer_info.peer_addr, peer_mac, ESP_NOW_ETH_ALEN);
    esp_now_peer_info.channel = ESP_CHANNEL;
    esp_now_peer_info.ifidx = ESP_IF_WIFI_STA;
    esp_now_add_peer(&esp_now_peer_info);

    return ESP_OK;
}

//************FUNCIÓN DE ENVIO DE DATOS POR ESPNOW*********************

static esp_err_t esp_now_send_data(const uint8_t *peer_addr, const uint8_t *data, size_t len)
{
    esp_now_send(peer_addr, data, len);
    return ESP_OK;
}

//*****************************TAREAS**********************************

void TareaEntradaDatos(void *Parametro)
{

    Data_IN DatosTX;

    while (1)
    {
        DatosTX.Sensor1 = gpio_get_level(IN1);
        DatosTX.Sensor2 = gpio_get_level(IN2);

        xQueueSend(xQueue, &DatosTX, portMAX_DELAY);

        vTaskDelay(200 / portTICK_PERIOD_MS);
    }

    /*
     Data_adc DatosTX;

     uint32_t promedio1 = 0;
     uint32_t promedio2 = 0;

     uint32_t adc_value1 = adc1_get_raw(CH1);
     uint32_t adc_value2 = adc1_get_raw(CH2);

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
         if (promedio1 > 512)
             promedio1 = 512;
         if (promedio2 > 512)
             promedio2 = 512;

         DatosTX.Sensor1 = promedio1;
         DatosTX.Sensor2 = promedio2;

         xQueueSend(xQueue, &DatosTX, portMAX_DELAY);

         vTaskDelay(200 / portTICK_PERIOD_MS);

}
*/
}

void ControlM(void *pvParameters)
{

    float error;
    float correccion;
    Data_IN DatosRX;

    while (1)
    {
        if (xQueueReceive(xQueue, &DatosRX, portMAX_DELAY) == pdPASS)
        {
            PID_Coefficients(p_pid_data, 0.0, CtrlData.KP, CtrlData.KI, 0.0);

            error = 10.0 * DatosRX.Sensor2 - 10.0 * DatosRX.Sensor1;
            correccion = PID_Update(p_pid_data, error);

            dutym1 = CtrlData.Vel - (correccion + CtrlData.offset);
            dutym2 = CtrlData.Vel + (correccion + CtrlData.offset);
            /*
                        if (dutym1 < 370)
                            dutym1 = 370;

                        if (dutym2 < 370)
                            dutym2 = 370;

                        if (dutym1 > 450)
                            dutym1 = 450;

                        if (dutym2 > 450)
                            dutym2 = 450;
                        // ESP_LOGI(TAG, "error:%f  coreccion: %f\n",error , correccion);
            */
            set_pwm(dutym1, dutym2, CtrlData.ON);
           

            if (CtrlData.ON)
            {
                if (CtrlData.Sentido1 == 0)
                {

                    gpio_set_level(OUT1, 0);
                    gpio_set_level(OUT2, 1);
                }
                else
                {
                    gpio_set_level(OUT1, 1);
                    gpio_set_level(OUT2, 0);
                }

                if (CtrlData.Sentido2 == 0)
                {

                    gpio_set_level(OUT3, 0);
                    gpio_set_level(OUT4, 1);
                }
                else
                {
                    gpio_set_level(OUT3, 1);
                    gpio_set_level(OUT4, 0);
                }
            }
            else
            {
                gpio_set_level(OUT1, 0);
                gpio_set_level(OUT2, 0);
                gpio_set_level(OUT3, 0);
                gpio_set_level(OUT4, 0);
            }

            ESP_LOGI(TAG, "%u|%u", dutym1, dutym2);

            gpio_set_level(LED2, CtrlData.ON);

            // vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }
}

void app_main(void)
{

    ESP_ERROR_CHECK(init_wifi());
    ESP_ERROR_CHECK(init_esp_now());
    ESP_ERROR_CHECK(register_peer(peer_mac));
    ESP_ERROR_CHECK(init_gpio());
    ESP_ERROR_CHECK(init_pwm());
    // ESP_ERROR_CHECK(init_adc());

    PID_Init(p_pid_data);
    xQueue = xQueueCreate(QUEUE_LENGTH, ITEM_SIZE);

    if (xQueue != NULL)
    {
        xTaskCreatePinnedToCore(TareaEntradaDatos, "Sigue linea", 1024 * 5, NULL, 1, NULL, 0);
        xTaskCreatePinnedToCore(ControlM, "PID", 1024 * 5, NULL, 1, NULL, 0);
    }
    else
    {
        ESP_LOGW(TAG, "Tareas no creadas");
    }
}
