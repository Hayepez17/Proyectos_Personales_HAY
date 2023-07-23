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
#include "init_gpio.h"
#include "init_adc.h"
#include "init_pwm.h"
#include "init_uart.h"
#include "tareas.h"

#define ESP_CHANNEL 1
#define sizeBUFF 50

extern UINT16_VAL MBHoldingRegister[maxHoldingRegister];
extern UINT16_VAL MBInputRegister[maxInputRegister];
extern UINT16_VAL MBCoils;
extern UINT16_VAL MBDiscreteInputs;

TaskHandle_t xHandle;
char BUFF[sizeBUFF];

//**********DIRECCIÓN MAC DEL RESPONDER************************

static uint8_t peer_mac[ESP_NOW_ETH_ALEN] = {0x94, 0xe6, 0x86, 0x01, 0xee, 0xe8};

static char *TAG = "esp_now_init";

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
    //ESP_LOGI(TAG, "Data recived " MACSTR " %s", MAC2STR(esp_now_info->src_addr), data);

    MBInputRegister[0].Val = atoi(strtok((char *)data, "|")); // "dutym1|dutym2|sensor1|sensor2" obtiene el primer valor de la cadena y guarda en variable
    MBInputRegister[1].Val = atoi(strtok(NULL, "|"));        // "dutym1|dutym2|sensor1|sensor2" obtiene el segundo valor de la cadena y guarda en variable
    
}

void send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    if (status == ESP_NOW_SEND_SUCCESS)
    {
       // ESP_LOGI(TAG, "ESP_NOW_SEND_SUCCESS");
    }
    else
    {
       // ESP_LOGW(TAG, "ESP_NOW_SEND_FAIL");
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

void app_main(void)
{
    ESP_ERROR_CHECK(init_wifi());
    ESP_ERROR_CHECK(init_esp_now());
    ESP_ERROR_CHECK(register_peer(peer_mac));

    initUART0(); // inicializo uart0
    //init_adc();  // inicializo ADCs
    init_gpio(); // inicializo salidas
    xTaskCreatePinnedToCore(Boton, "Boton", 1024, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(SendDatos, "Envio ESPNOW", 1024 * 5, NULL, 1, NULL, 0);
    //xTaskCreatePinnedToCore(TareaEntradaDatos, "Tarea_para_entrada1", 1024 * 5, NULL, 1, NULL, 1); // creo tarea1
}

//*********************FUNCIÓN DE TAREAS*************************

void Boton(void *pvParameters)
{

    while (1)
    {
        // presiona boton y suelta
        while (gpio_get_level(B1) == 0)
            vTaskDelay(20 / portTICK_PERIOD_MS);

        while (gpio_get_level(B1) == 1)
            vTaskDelay(20 / portTICK_PERIOD_MS);

        MBCoils.bits.b0 = !MBCoils.bits.b0;
        gpio_set_level(LED2, MBCoils.bits.b0);

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void SendDatos(void *pvParameters)
{

    while (1)
    {
 
        snprintf(BUFF, sizeof(BUFF), "%i|%u|%u|%u|%u|%u|%i|%i", MBCoils.bits.b0, MBHoldingRegister[2].Val, MBHoldingRegister[3].Val, MBHoldingRegister[4].Val, MBHoldingRegister[0].Val, MBHoldingRegister[1].Val, MBCoils.bits.b1, MBCoils.bits.b2);  // Arreglo cadena de caracteres en forma "ON|tiempo_led"
        esp_now_send_data(peer_mac, (const uint8_t *)BUFF, sizeBUFF); // Envía cadena en forma "ON|Vel|Kp|Ki"
    

    gpio_set_level(LED2, MBCoils.bits.b0);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}