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
#include "tareas.h"
#include "ControlPID.h"

#define ESP_CHANNEL 1

TaskHandle_t xHandle;

// Queue parameters
#define QUEUE_LENGTH 1
#define ITEM_SIZE sizeof(Datos)

// Queue handle
QueueHandle_t xQueue;

char BUFF[32];

typedef struct
{
    bool ON;
    uint16_t Vel;
    float KP;
    float KI;
    uint16_t Sensor1;
    uint16_t Sensor2;
    int offset;
} Datos;

static uint8_t peer_mac[ESP_NOW_ETH_ALEN] = {0xc8, 0xf0, 0x9e, 0xf1, 0x6b, 0x10};

static char *TAG = "esp_now_resp";

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
    Datos datosTx;

    ESP_LOGI(TAG, "Data recived " MACSTR " %s", MAC2STR(esp_now_info->src_addr), data);

    datosTx.ON = atoi(strtok((char *)data, "|"));  // "ON|Vel|KP|KI|S1|S2|offset" obtiene el primer valor de la cadena y guarda en variable
    datosTx.Vel = atoi(strtok(NULL, "|"));         // "ON|Vel|KP|KI|S1|S2|offset" obtiene el segundo valor de la cadena y guarda en variable
    datosTx.KP = atoi(strtok(NULL, "|")) / 100.0f; // "ON|Vel|KP|KI|S1|S2|offset" obtiene el tercer valor de la cadena y guarda en variable
    datosTx.KI = atoi(strtok(NULL, "|")) / 100.0f; // "ON|Vel|KP|KI|S1|S2|offset" obtiene el cuarto valor de la cadena y guarda en variable
    datosTx.Sensor1 = atoi(strtok(NULL, "|"));     // "ON|Vel|KP|KI|S1|S2|offset" obtiene el quinto valor de la cadena y guarda en variable
    datosTx.Sensor2 = atoi(strtok(NULL, "|"));     // "ON|Vel|KP|KI|S1|S2|offset" obtiene el sexto valor de la cadena y guarda en variable
    datosTx.offset = atoi(strtok(NULL, "|"));     // "ON|Vel|KP|KI|S1|S2|offset" obtiene el septimo valor de la cadena y guarda en variable

    xQueueSend(xQueue, &datosTx, portMAX_DELAY);
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

//************FUNCIÓN DE INICIALIZACIÓN DE ESPNOW*******************

static esp_err_t init_esp_now(void)
{
    esp_now_init();
    esp_now_register_recv_cb(recv_cb);
    esp_now_register_send_cb(send_cb);

    ESP_LOGI(TAG, "esp now init completed");
    return ESP_OK;
}

//******************FUNCIÓN DE REGISTRO ESPNOW************************

static esp_err_t register_peer(uint8_t *peer_addr)
{
    esp_now_peer_info_t esp_now_peer_info = {};
    memcpy(esp_now_peer_info.peer_addr, peer_mac, ESP_NOW_ETH_ALEN);
    esp_now_peer_info.channel = ESP_CHANNEL;
    esp_now_peer_info.ifidx = ESP_IF_WIFI_STA;
    esp_now_add_peer(&esp_now_peer_info);

    return ESP_OK;
}

//************FUNCIÓN DE ENVIO DE DATOS POR ESPNOW***************

static esp_err_t esp_now_send_data(const uint8_t *peer_addr, const uint8_t *data, size_t len)
{
    esp_now_send(peer_addr, data, len);
    return ESP_OK;
}

void ControlM(void *pvParameters)
{
    Datos datosRx;
    float error;
    float correccion;

    while (1)
    {
        if (xQueueReceive(xQueue, &datosRx, portMAX_DELAY) == pdPASS)
        {
            PID_Coefficients(p_pid_data, 0.0, datosRx.KP, datosRx.KI, 0.0);

            error = datosRx.Sensor1 - datosRx.Sensor2;
            correccion = PID_Update(p_pid_data, error);

            uint16_t dutym1 = datosRx.Vel - (correccion + datosRx.offset);
            uint16_t dutym2 = datosRx.Vel + (correccion + datosRx.offset);

           // ESP_LOGI(TAG, "error:%f  coreccion: %f\n",error , correccion);

            set_pwm(dutym1, dutym2, datosRx.ON);

            snprintf(BUFF, sizeof(BUFF), "%u|%u", dutym1, dutym2);  // Arreglo cadena de caracteres en forma "dutym1|dutym2"
            esp_now_send_data(peer_mac, (const uint8_t *)BUFF, 32); // Envía cadena en forma "dutym1|dutym2"

            ESP_LOGI(TAG,"%u|%u", dutym1, dutym2);

            gpio_set_level(LED2, datosRx.ON);

            //vTaskDelay(100 / portTICK_PERIOD_MS);
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

    PID_Init(p_pid_data);
    xQueue = xQueueCreate(QUEUE_LENGTH, ITEM_SIZE);

    if (xQueue != NULL)
    {
        xTaskCreatePinnedToCore(ControlM, "Parpadeo", 1024 * 5, NULL, 12, &xHandle, 0);
    }
    else
    {
        ESP_LOGW(TAG, "Tareas no creadas");
    }
}
