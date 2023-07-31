#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <math.h>

#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_now.h"
#include "esp_mac.h"
#include <esp_timer.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "init_gpio.h"
#include "init_pwm.h"
#include "init_adc.h"
#include "tareas.h"
#include "ControlPID.h"

#define ESP_CHANNEL 1
#define VelSpinR 165
#define VelSpinL 200
#define Dutyarranque 220
#define max_time_us 100000
#define offset -13
#define timeout_expired(start, len) ((esp_timer_get_time() - (start)) >= (len))

// Queue parameters
#define ITEM_SIZE sizeof(Datos)
#define QUEUE_LENGTH 1

QueueHandle_t xQueue;

TaskHandle_t xHandle;

// buffer esp now send

char BUFF[32];

// Mac addres peer esp now

// static uint8_t peer_mac[ESP_NOW_ETH_ALEN] = {0xc8, 0xf0, 0x9e, 0xf1, 0x6b, 0x10};
static uint8_t peer_mac[ESP_NOW_ETH_ALEN] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

// etiquetas

static char *TAG = "esp_now_resp";

// Estructuras y variables globales

typedef struct
{
    bool ON;
    uint16_t VelMax;
    uint16_t VelMin;
    float KV;
    float KP;
    float KI;
    bool B_RIGHT;
    bool B_LEFTH;
} Datos;

Datos CtrlData;

int16_t dutym1;
int16_t dutym2;
int16_t reposo;

const float EULER = 2.718281828459045;
//******************PROTOTIPO DE FUNCIONES********************

static esp_err_t esp_now_send_data(const uint8_t *peer_addr, const uint8_t *data, size_t len);
void SetDirecction(int x);
void arranque(uint16_t DutyNom1, uint16_t DutyNom2);

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

    CtrlData.ON = atoi(strtok((char *)data, "|"));   // "ON|Vel|KP|KI|S1|S2|offset" obtiene el primer valor de la cadena y guarda en variable
    CtrlData.VelMax = atoi(strtok(NULL, "|"));       // "ON|Vel|KP|KI|S1|S2|offset" obtiene el segundo valor de la cadena y guarda en variable
    CtrlData.VelMin = atoi(strtok(NULL, "|"));       // "ON|Vel|KP|KI|S1|S2|offset" obtiene el segundo valor de la cadena y guarda en variable
    CtrlData.KV = atoi(strtok(NULL, "|")) / 1000.0f; // "ON|Vel|KP|KI|S1|S2|offset" obtiene el tercer valor de la cadena y guarda en variable
    CtrlData.KP = atoi(strtok(NULL, "|")) / 100.0f;  // "ON|Vel|KP|KI|S1|S2|offset" obtiene el tercer valor de la cadena y guarda en variable
    CtrlData.KI = atoi(strtok(NULL, "|")) / 1000.0f; // "ON|Vel|KP|KI|S1|S2|offset" obtiene el cuarto valor de la cadena y guarda en variable
    CtrlData.B_LEFTH = atoi(strtok(NULL, "|"));      // "ON|Vel|KP|KI|S1|S2|offset" obtiene el quinto valor de la cadena y guarda en variable
    CtrlData.B_RIGHT = atoi(strtok(NULL, "|"));      // "ON|Vel|KP|KI|S1|S2|offset" obtiene el sexto valor de la cadena y guarda en variable

    snprintf(BUFF, sizeof(BUFF), "%u|%u", dutym1, dutym2);  // Arreglo cadena de caracteres en forma "dutym1|dutym2"
    esp_now_send_data(peer_mac, (const uint8_t *)BUFF, 32); // Envía cadena en forma "dutym1|dutym2"

    if (CtrlData.ON)
    {
        vTaskResume(xHandle);
    }

    gpio_set_level(LED2, CtrlData.ON);
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
    int64_t time_start;

    while (1)
    {

        /*
                if (DatosTX.Sensor1 == 1 && DatosTX.Sensor2 == 1)
                {
                    gpio_set_level(OUT1, 0);
                    gpio_set_level(OUT2, 0);
                    gpio_set_level(OUT3, 0);
                    gpio_set_level(OUT4, 0);

                    vTaskDelay(1 / portTICK_PERIOD_MS);
                }
                else
                    xQueueSend(xQueue, &DatosTX, portMAX_DELAY);
        */
        if (gpio_get_level(IN1) == 1 && gpio_get_level(IN2) == 1 && CtrlData.KP != 0)
        {
            SetDirecction(STOP);
            reposo = 1;
            vTaskDelay(20 / portTICK_PERIOD_MS);
        }
        else
        {

            DatosTX.Sensor1 = gpio_get_level(IN1);
            DatosTX.Sensor2 = gpio_get_level(IN2);
            DatosTX.F_LEFT = 0;
            DatosTX.F_LEFT = 0;
            xQueueSend(xQueue, &DatosTX, portMAX_DELAY);
            /*
                        time_start = esp_timer_get_time();
                        while (gpio_get_level(IN1) && CtrlData.KP != 0 && DatosTX.F_LEFT == 0)
                        {
                            if (timeout_expired(time_start, max_time_us))
                            {
                                SetDirecction(STOP);
                                vTaskDelay(1000 / portTICK_PERIOD_MS);
                                DatosTX.F_LEFT = 1;
                                break;
                            }

                            DatosTX.Sensor1 = gpio_get_level(IN1);
                            DatosTX.Sensor2 = gpio_get_level(IN2);
                            xQueueSend(xQueue, &DatosTX, portMAX_DELAY);
                        }

                        time_start = esp_timer_get_time();
                        while (gpio_get_level(IN2) && CtrlData.KP != 0 && DatosTX.F_RIGHT == 0)
                        {
                            if (timeout_expired(time_start, max_time_us))
                            {
                                SetDirecction(STOP);
                                vTaskDelay(1000 / portTICK_PERIOD_MS);
                                DatosTX.F_RIGHT = 1;
                                break;
                            }

                            DatosTX.Sensor1 = gpio_get_level(IN1);
                            DatosTX.Sensor2 = gpio_get_level(IN2);
                            xQueueSend(xQueue, &DatosTX, portMAX_DELAY);
                        }

                        */
        }
    }
}

void ControlM(void *pvParameters)
{

    float error;
    float correccion;
    float velM;
    float pot;
    Data_IN DatosRX;
    while (1)
    {
        if (xQueueReceive(xQueue, &DatosRX, portMAX_DELAY) == pdPASS)
        {

            if (CtrlData.ON)
            {
                /*if (DatosRX.F_LEFT)
                {
                    set_pwm(0, 160);
                    SetDirecction(FORWARD);
                    vTaskDelay(500 / portTICK_PERIOD_MS);
                    SetDirecction(STOP);
                    DatosRX.F_LEFT = 0;
                }

                if (DatosRX.F_RIGHT)
                {
                    set_pwm(192, 0);
                    SetDirecction(FORWARD);
                    vTaskDelay(500 / portTICK_PERIOD_MS);
                    SetDirecction(STOP);
                    DatosRX.F_RIGHT = 0;
                }
*/
                PID_Coefficients(p_pid_data, 0.0, CtrlData.KP, CtrlData.KI, 0.0);

                error = 10.0 * DatosRX.Sensor2 - 10.0 * DatosRX.Sensor1;
                correccion = PID_Update(p_pid_data, error);

                pot = -1.0 * CtrlData.KV * fabs(error*CtrlData.KP);
                velM = CtrlData.VelMin + (CtrlData.VelMax - CtrlData.VelMin) * pow(EULER, pot);

                dutym1 = velM - (correccion + offset);
                dutym2 = velM + (correccion + offset);

                if (dutym1 < 50)
                    dutym1 = 0;
                if (dutym2 < 50)
                    dutym2 = 0;

                if (dutym1 != 0 && dutym2 != 0 && reposo == 1)
                {
                    arranque(dutym1, dutym2);
                }

                else
                    set_pwm(dutym1, dutym2);

                SetDirecction(FORWARD);
                /*if (DatosRX.Sensor2 == 1 && DatosRX.Sensor1 == 1 && CtrlData.KP != 0)
                {
                    gpio_set_level(OUT1, 0);
                    gpio_set_level(OUT2, 0);
                    gpio_set_level(OUT3, 0);
                    gpio_set_level(OUT4, 0);
                }

                else
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
                */
            }
            else
            {
                SetDirecction(STOP);
                set_pwm(0, 0);
                vTaskSuspend(xHandle);
                reposo = 1;
            }

            // ESP_LOGI(TAG, "%u|%u", dutym1, dutym2);

            // vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }
}

void SetDirecction(int x)
{
    switch (x)
    {

    case STOP:

        gpio_set_level(OUT1, 0);
        gpio_set_level(OUT2, 0);
        gpio_set_level(OUT3, 0);
        gpio_set_level(OUT4, 0);

        break;

    case FORWARD:
        gpio_set_level(OUT1, 0);
        gpio_set_level(OUT2, 1);
        gpio_set_level(OUT3, 1);
        gpio_set_level(OUT4, 0);
        break;

    case LEFT:
        gpio_set_level(OUT1, 1);
        gpio_set_level(OUT2, 0);
        gpio_set_level(OUT3, 1);
        gpio_set_level(OUT4, 0);
        break;

    case RIGHT:
        gpio_set_level(OUT1, 0);
        gpio_set_level(OUT2, 1);
        gpio_set_level(OUT3, 0);
        gpio_set_level(OUT4, 1);
        break;

    case BACK:
        gpio_set_level(OUT1, 1);
        gpio_set_level(OUT2, 0);
        gpio_set_level(OUT3, 0);
        gpio_set_level(OUT4, 1);
        break;

    default:
        break;
    }
}

void arranque(uint16_t DutyNom1, uint16_t DutyNom2)
{
    if (DutyNom1 == 0)
        set_pwm(0, Dutyarranque);
    else
    {
        if (DutyNom2 == 0)
            set_pwm(Dutyarranque, 0);
        else
        {
            set_pwm(Dutyarranque, Dutyarranque);
            vTaskDelay(500 / portTICK_PERIOD_US);
            set_pwm(DutyNom1, DutyNom2); /* code */
            reposo = 0;
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
    SetDirecction(STOP);
    PID_Init(p_pid_data);
    xQueue = xQueueCreate(QUEUE_LENGTH, ITEM_SIZE);
    reposo = 1;

    if (xQueue != NULL)
    {
        xTaskCreatePinnedToCore(TareaEntradaDatos, "Sigue linea", 1024 * 5, NULL, 1, &xHandle, 0);
        xTaskCreatePinnedToCore(ControlM, "PID", 1024 * 10, NULL, 1, NULL, 1);
    }
    else
    {
        ESP_LOGW(TAG, "Tareas no creadas");
    }
}
