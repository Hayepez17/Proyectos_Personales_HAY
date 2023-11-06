#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include <esp_http_server.h>
#include "nvs_flash.h"
#include "esp_spiffs.h"
#include "init_gpio.h"
#include "connect_wifi.h"
#include "init_adc.h"
#include "tareas.h"
#include "ultrasonic.h"
#include "init_pwm.h"
#include "ControlPID.h"

// PI AND PWM PARAMETERS

#define Dutyarranque 220
#define offset -5
#define VelMax 180
#define VelMin 165

const float KP = 3.5;
const float KI = 0.001;
const float KV = 0.04;

// ULTRASONIC PARAMETERS

#define TRIGGER_GPIO GPIO_NUM_32
#define ECHO_GPIO GPIO_NUM_26
#define MinimunDistanceCM 15

// VARIABLES GLOBALES

bool weigth;
bool cruce;
bool obstaculo;
bool state_mode;
bool reposo;
bool adelante;

int tiempo_led;
int rssi;

uint32_t UltraData;

const float EULER = 2.718281828459045;

// ESTRUCURAS GLOBALES

httpd_handle_t server = NULL;
struct async_resp_arg
{
    httpd_handle_t hd;
    int fd;
};

DatosDpad Dpad;
// Data_IN SigueLinea;

// QUEUE PARAMETERS

#define ITEM_SIZE1 sizeof(DatosDpad)
#define ITEM_SIZE2 sizeof(Data_IN)
#define QUEUE_LENGTH 1

QueueHandle_t xQueue1;
// QueueHandle_t xQueue2;

// TAGS

static const char *TAG = "WebSocket Server"; // TAG for debug

// SPIFFS PARAMETERS

#define INDEX_HTML_PATH "/spiffs/index.html"
char index_html[4096 * 10];
char response_data[4096 * 10];

/***************************LECTURA DATA PAGINA WEB***********************************/

static void initi_web_page_buffer(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true};

    ESP_ERROR_CHECK(esp_vfs_spiffs_register(&conf));

    memset((void *)index_html, 0, sizeof(index_html));
    struct stat st;
    if (stat(INDEX_HTML_PATH, &st))
    {
        ESP_LOGE(TAG, "index.html not found");
        return;
    }

    FILE *fp = fopen(INDEX_HTML_PATH, "r");
    if (fread(index_html, st.st_size, 1, fp) == 0)
    {
        ESP_LOGE(TAG, "fread failed");
    }
    fclose(fp);
}

/****************************FUNCIONES WEBSOCKET***********************************/

esp_err_t get_req_handler(httpd_req_t *req)
{
    int response;

    sprintf(response_data, index_html);

    response = httpd_resp_send(req, response_data, HTTPD_RESP_USE_STRLEN);
    return response;
}

static void ws_async_send(void *arg)
{
    uint16_t value1;
    httpd_ws_frame_t ws_pkt;
    struct async_resp_arg *resp_arg = arg;
    httpd_handle_t hd = resp_arg->hd;
    int fd = resp_arg->fd;

    if (weigth)
        value1 = InDataADC();
    else
        value1 = 0;

    char buff[256];
    memset(buff, 0, sizeof(buff));
    sprintf(buff, "{\"value1\":%u,\"StateSwitch\":%u,\"StateButton\":%u,\"DirectionButton\":%i,\"StateWeigth\":%u}", value1, state_mode, Dpad.state_button, Dpad.direction_button, weigth);

    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.payload = (uint8_t *)buff;
    ws_pkt.len = strlen(buff);
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;

    static size_t max_clients = CONFIG_LWIP_MAX_LISTENING_TCP;
    size_t fds = max_clients;
    int client_fds[max_clients];

    esp_err_t ret = httpd_get_client_list(server, &fds, client_fds);

    if (ret != ESP_OK)
    {
        return;
    }

    for (int i = 0; i < fds; i++)
    {
        int client_info = httpd_ws_get_fd_info(server, client_fds[i]);
        if (client_info == HTTPD_WS_CLIENT_WEBSOCKET)
        {
            httpd_ws_send_frame_async(hd, client_fds[i], &ws_pkt);
        }
    }
    free(resp_arg);
}

static esp_err_t trigger_async_send(httpd_handle_t handle, httpd_req_t *req)
{
    struct async_resp_arg *resp_arg = malloc(sizeof(struct async_resp_arg));
    resp_arg->hd = req->handle;
    resp_arg->fd = httpd_req_to_sockfd(req);
    return httpd_queue_work(handle, ws_async_send, resp_arg);
}

static esp_err_t handle_ws_req(httpd_req_t *req)
{
    if (req->method == HTTP_GET)
    {
        ESP_LOGI(TAG, "Handshake done, the new connection was opened");
        return ESP_OK;
    }

    httpd_ws_frame_t ws_pkt;
    uint8_t *buf = NULL;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "httpd_ws_recv_frame failed to get frame len with %d", ret);
        return ret;
    }

    if (ws_pkt.len)
    {
        buf = calloc(1, ws_pkt.len + 1);
        if (buf == NULL)
        {
            ESP_LOGE(TAG, "Failed to calloc memory for buf");
            return ESP_ERR_NO_MEM;
        }
        ws_pkt.payload = buf;
        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "httpd_ws_recv_frame failed with %d", ret);
            free(buf);
            return ret;
        }
        ESP_LOGI(TAG, "Got packet with message: %s", ws_pkt.payload);
    }

    ESP_LOGI(TAG, "frame len is %d", ws_pkt.len);

    if (ws_pkt.type == HTTPD_WS_TYPE_TEXT &&
        strcmp((char *)ws_pkt.payload, "MANUAL") == 0)
    {
        set_mode((char *)ws_pkt.payload);
        printf((char *)ws_pkt.payload);
        free(buf);
        return trigger_async_send(req->handle, req);
    }

    if (ws_pkt.type == HTTPD_WS_TYPE_TEXT &&
        strcmp((char *)ws_pkt.payload, "AUTO") == 0)
    {
        set_mode((char *)ws_pkt.payload);
        printf((char *)ws_pkt.payload);
        free(buf);
        return trigger_async_send(req->handle, req);
    }

    if (ws_pkt.type == HTTPD_WS_TYPE_TEXT && ws_pkt.len > 6)
    {
        if (strcmp((char *)ws_pkt.payload, "WEIGTH-UPDATE") == 0)
        {
            weigth = 1;
            printf((char *)ws_pkt.payload);
            free(buf);
            return trigger_async_send(req->handle, req);
        }
        if (strcmp((char *)ws_pkt.payload, "WEIGTH-OFF") == 0)
        {
            weigth = 0;
            printf((char *)ws_pkt.payload);
            free(buf);
            return trigger_async_send(req->handle, req);
        }
    }

    if (ws_pkt.type == HTTPD_WS_TYPE_TEXT && ws_pkt.len < 4)
    {
        char mnsg[4];
        memset(mnsg, 0, sizeof(mnsg));
        printf((char *)ws_pkt.payload);
        sprintf(mnsg, (char *)ws_pkt.payload);
        Dpad.direction_button = atoi(strtok(mnsg, "|"));
        Dpad.state_button = atoi(strtok(NULL, "|"));

        free(buf);
        // xQueueSend(xQueue1, &DatosTx, portMAX_DELAY);
        return trigger_async_send(req->handle, req);
    }
    return ESP_OK;
}

httpd_handle_t setup_websocket_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    httpd_uri_t uri_get = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = get_req_handler,
        .user_ctx = NULL};

    httpd_uri_t ws = {
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = handle_ws_req,
        .user_ctx = NULL,
        .is_websocket = true};

    if (httpd_start(&server, &config) == ESP_OK)
    {
        httpd_register_uri_handler(server, &uri_get);
        httpd_register_uri_handler(server, &ws);
    }

    return server;
}

/****************************FUNCIONES DE LLAMADA**********************************/

uint16_t InDataADC(void)
{
    uint16_t promedio1 = 0;

    uint16_t adc_value1 = adc1_get_raw(CH1);

    for (int j = 0; j < NumeroMuestras; j++)
    {
        promedio1 += adc_value1 = adc1_get_raw(CH1);
    }
    // Calculo Promedio de 100 muestras
    promedio1 /= NumeroMuestras;

    // Establezco límite superior de entrada
    if (promedio1 > 512)
        promedio1 = 512;

    return promedio1;
}

void set_mode(const char *MODE)
{
    // Datos DatosTx;
    // Data_IN DatosRx;

    switch (MODE[0])
    {
    case 'M':
        state_mode = 0;
        tiempo_led = 1000;
        SetDirecction(STOP);
        /*  if (uxQueueSpacesAvailable(xQueue2))
          {
              xQueueReceive(xQueue2,&DatosRx,portMAX_DELAY);
          }
          vTaskSuspend(xHandle3);
          */
        gpio_set_level(LED2, 1);
        printf(MODE);
        // xQueueSend(xQueue1, &DatosTx, portMAX_DELAY);

        break;

    case 'A':
        state_mode = 1;
        weigth = 0;
        tiempo_led = 500;

        // vTaskResume(xHandle3);
        printf(MODE);
        break;

    default:
        break;
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
        set_pwm(0, 0);
        reposo = 1;
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
    if (reposo)
    {

        set_pwm(Dutyarranque, Dutyarranque);
        SetDirecction(FORWARD);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        set_pwm(DutyNom1, DutyNom2); /* code */
        reposo = 0;
    }
    else
        set_pwm(DutyNom1, DutyNom2);
}

/************************************TAREAS****************************************/

void BlinkLed(void *pvParameters)
{
    int ON = 0;

    while (1)
    {
        if (state_mode == 1 || obstaculo == 1)
        {
            ON = !ON;
            gpio_set_level(LED2, ON);
        }
        vTaskDelay(tiempo_led / portTICK_PERIOD_MS);
    }
}

void DatosUltra(void *pvParameters)
{
    ultrasonic_sensor_t sensorU = {
        .trigger_pin = TRIGGER_GPIO,
        .echo_pin = ECHO_GPIO};

    wifi_ap_record_t ap;

    uint32_t distance;
    ultrasonic_init(&sensorU);

    while (1)
    {
        if (state_mode == 1 || adelante == 1)
        {
            esp_err_t res = ultrasonic_measure_cm(&sensorU, MAX_DISTANCE_CM, &distance);
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
                    distance = MAX_DISTANCE_CM;
                    break;
                default:
                    printf("%s\n", esp_err_to_name(res));
                }
            }
            // printf("Distance: %lu cm\n", distance);
            UltraData = distance;

            if (state_mode)
            {
                esp_wifi_sta_get_ap_info(&ap);
                rssi = ap.rssi;
            }

            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        else
            vTaskDelay(1500 / portTICK_PERIOD_MS);
    }
}
/*
void DataIn(void *pvParameters)
{

    Data_IN DatosTX;
    // int64_t time_start;

    while (1)
    {
        if (state_mode)
        {
            DatosTX.Sensor1 = gpio_get_level(IN1);
            DatosTX.Sensor2 = gpio_get_level(IN2);

            if ((DatosTX.Sensor1 == 1 && DatosTX.Sensor2 == 1) || UltraData <= MinimunDistanceCM)
            {

                tiempo_led = 50;
                cruce = 1;
                SetDirecction(STOP);
                vTaskDelay(200 / portTICK_PERIOD_MS);
            }
            else
            {
                tiempo_led = 500;
                cruce = 0;
                xQueueSend(xQueue1, &DatosTX, portMAX_DELAY);
                vTaskDelay(10 / portTICK_PERIOD_MS);
            }
        }
        else
            vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
*/
void ControlM(void *pvParameters)
{

    while (1)
    {

        if (!state_mode)
        {
            if (Dpad.state_button)
            {
                if (Dpad.direction_button == 0)
                {
                    adelante = 1;
                    // printf("ultra:%lu", UltraData);
                    if ((UltraData > MinimunDistanceCM))
                    {
                        obstaculo = 0;
                        tiempo_led = 100;
                        SetDirecction(FORWARD);

                        arranque(200 - offset, 200 + offset);
                    }
                    else
                    {
                        obstaculo = 1;
                        tiempo_led = 50;
                        SetDirecction(STOP);
                    }
                }

                if (Dpad.direction_button == 1)
                {
                    SetDirecction(LEFT);
                    arranque(200 - offset, 200 + offset);
                }

                if (Dpad.direction_button == 2)
                {
                    SetDirecction(BACK);
                    arranque(200 - offset, 200 + offset);
                }

                if (Dpad.direction_button == 3)
                {
                    SetDirecction(RIGHT);
                    arranque(200 - offset, 200 + offset);
                }

                vTaskDelay(10 / portTICK_PERIOD_MS);
            }
            else
            {
                tiempo_led = 1000;
                obstaculo = 0;
                adelante = 0;
                gpio_set_level(LED2, 1);
                SetDirecction(STOP);
                vTaskDelay(10 / portTICK_PERIOD_MS);
            }
        }
        else
            vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void ControlA(void *pvParameters)
{
    float error;
    float correccion;
    float velM;
    float pot;

    bool sensor1 = 0;
    bool sensor2 = 0;

    // Data_IN DatosRX;

    while (1)
    {

        if (state_mode)
        {
            if (rssi <= -40)
            {

                sensor2 = gpio_get_level(IN1);
                vTaskDelay(100 / portTICK_PERIOD_US);
                sensor1 = gpio_get_level(IN2);

                if ((sensor1 == 1 && sensor2 == 1) || UltraData <= MinimunDistanceCM)
                {
                    tiempo_led = 50;
                    // cruce = 1;
                    SetDirecction(STOP);
                    vTaskDelay(200 / portTICK_PERIOD_MS);
                }
                else
                {
                    PID_Coefficients(p_pid_data, 0.0, KP, KI, 0.0);
                    tiempo_led = 500;
                    error = 10.0 * sensor1 - 10.0 * sensor2;
                    correccion = PID_Update(p_pid_data, error);

                    pot = -1.0 * KV * fabs(error * KP);
                    velM = VelMin + (VelMax - VelMin) * pow(EULER, pot);

                    uint8_t dutym1 = velM - (correccion + offset);
                    uint8_t dutym2 = velM + (correccion + offset);

                    if (dutym1 < 50)
                        dutym1 = 0;
                    if (dutym2 < 50)
                        dutym2 = 0;

                    // sprintf("dutym1: %u, dutym2: %u \n", dutym1, dutym2);
                    arranque(dutym1, dutym2);

                    SetDirecction(FORWARD);

                    printf("%d, %d, %i \n", dutym1, dutym2, rssi);

                    vTaskDelay(10 / portTICK_PERIOD_MS);
                }
            }
            else
            {
                if (UltraData <= MinimunDistanceCM)
                    tiempo_led = 50;
                else
                    tiempo_led = 500;
                SetDirecction(STOP);
                printf("rssi %i \n", rssi);
                vTaskDelay(100 / portTICK_PERIOD_MS);
            }
        }
        else
            vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

/*************************************MAIN*****************************************/

void app_main()
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    connect_wifi();

    if (wifi_connect_status)
    {
        ESP_LOGI(TAG, "ESP32 ESP-IDF WebSocket Web Server is running ... ...\n");

        ESP_ERROR_CHECK(init_gpio());
        ESP_ERROR_CHECK(init_adc());
        ESP_ERROR_CHECK(init_pwm());
        PID_Init(p_pid_data);
        cruce = 0;
        obstaculo = 0;
        set_mode("MANUAL");

        xQueue1 = xQueueCreate(QUEUE_LENGTH, ITEM_SIZE2);

        if (xQueue1 != NULL)
        {
            xTaskCreatePinnedToCore(BlinkLed, "led_mode", 1024 * 2, NULL, 1, NULL, 1);
            xTaskCreatePinnedToCore(DatosUltra, "Actualización de lectura ultrasonido", 1024 * 5, NULL, 1, NULL, 1);
            xTaskCreatePinnedToCore(ControlM, "control modo manual", 1024 * 10, NULL, 1, NULL, 1);
            xTaskCreatePinnedToCore(ControlA, "control modo automatico", 1024 * 10, NULL, 1, NULL, 0);
            // xTaskCreatePinnedToCore(DataIn, "Datos entrada control", 1024 * 5, NULL, 1, NULL, 0);

            initi_web_page_buffer();
            setup_websocket_server();
        }
        else
        {
            ESP_LOGW(TAG, "Tareas no creadas");
        }
    }
}