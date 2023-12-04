#include <Arduino.h>
#include "SPIFFS.h"
#include "freertos/queue.h"
#include "init_adc.h"
#include "init_gpio.h"
#include "tareas.h"
#include "ultrasonic.h"
#include <WiFiClientSecure.h>
#include <Wire.h>
#include <PubSubClient.h>

// Credenciales de red Wifi
const char *ssid = "wifihector";
const char *password = "123456789";

// Servidor MQTT
const char *mqtt_server = "a1vreyb61dmm02-ats.iot.us-east-1.amazonaws.com";
const int mqtt_port = 8883;

String Read_rootca;
String Read_cert;
String Read_privatekey;
//********************************
#define BUFFER_LEN 256
char msg[BUFFER_LEN];
int value = 0;
byte mac[6];
char mac_Id[18];
int count = 1;
//***************Ultrasonic Parameters*****************

ultrasonic_sensor_t sensorU = {
    .trigger_pin = TRIGGER_GPIO,
    .echo_pin = ECHO_GPIO};

//********************************

// Queue parameters
#define QUEUE_LENGTH 1
#define ITEM_SIZE sizeof(Datos)

// Queue handle
QueueHandle_t xQueue;

typedef struct
{
  float valuef;
  uint32_t valadc1;
  uint32_t valadc2;
} Datos;

//********************************

// Configuración de cliente MQTT
WiFiClientSecure espClient;
PubSubClient client(espClient);

// Conectar a red Wifi
void setup_wifi()
{
  vTaskDelay(pdMS_TO_TICKS(10));
  Serial.println();
  Serial.print("Conectando.. ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    vTaskDelay(pdMS_TO_TICKS(500));
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi conectado");
  Serial.println("Direccion IP: ");
  Serial.println(WiFi.localIP());
}

// Callback
void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Mensaje recibido [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

// Conectar a broker MQTT
void reconnect()
{

  // Loop para reconección
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");

    // Creando un ID como ramdon
    String clientId = "ESP32-";
    clientId += String(random(0xffff), HEX);

    // Intentando conectarse
    if (client.connect(clientId.c_str()))
    {
      Serial.println("conectada");

      // Conectado, publicando un payload...
      client.publish("ei_out", "hello world");

      // ... y suscribiendo
      client.subscribe("ei_in");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" Esperando 5 segundos");

      // Tiempo muerto de 5 segundos
      vTaskDelay(pdMS_TO_TICKS(5000));
    }
  }
}

// Tareas
void EntradaDatos(void *pvParameters)
{
  uint32_t promedio1 = 0;
  uint32_t promedio2 = 0;

  Datos datosTx;

  uint32_t adc_value1 = adc1_get_raw(CH1);
  uint32_t adc_value2 = adc1_get_raw(CH2);

  while (1)
  {
    // Iteración valores ADCs
    for (int j = 0; j < NumeroMuestras; j++)
    {

      promedio1 += adc_value1 = adc1_get_raw(CH1);
      promedio2 += adc_value2 = adc1_get_raw(CH2);
    }

    vTaskDelay(20 / portTICK_PERIOD_MS);

    // Calculo Promedio de 100 muestras
    promedio1 /= NumeroMuestras;
    promedio2 /= NumeroMuestras;

    // Establezco límite superior de entrada
    if (promedio1 > 512)
      promedio1 = 512;
    if (promedio2 > 512)
      promedio2 = 512;

    datosTx.valadc1 = promedio1;
    datosTx.valadc2 = promedio2;
    datosTx.valuef = Ultrasonic();

    xQueueSend(xQueue, &datosTx, portMAX_DELAY);
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

void SalidaDatos(void *pvParameters)
{
  Datos datosRx;

  while (1)
  {
    if (xQueueReceive(xQueue, &datosRx, portMAX_DELAY) == pdPASS)
    {
      String macIdStr = mac_Id;
      String Sensor1 = String(datosRx.valadc1);
      String Sensor2 = String(datosRx.valadc2);
      String Sensor3 = String(datosRx.valuef);
      snprintf(msg, BUFFER_LEN, "{\"mac_Id\" : \"%s\", \"Sensor1\" : %s, \"Sensor2\" : %s, \"Sensor3\" : %s}", macIdStr.c_str(), Sensor1.c_str(), Sensor2.c_str(), Sensor3.c_str());
      Serial.print("Publicando mensaje: ");
      Serial.print(count);
      Serial.println(msg);
      client.publish("sensor", msg);
      count++;

      vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
  }
}

void EnviarAlerta(void *pvParameters)
{
  Datos datosTx;

  while (1)
  {
    // presiona boton y suelta
    while (gpio_get_level(BT) == 0)
      vTaskDelay(100 / portTICK_PERIOD_MS); // boton presionado

    while (gpio_get_level(BT) == 1)
      vTaskDelay(100 / portTICK_PERIOD_MS); // boton sin presionar

    datosTx.valadc1 = 710;
    datosTx.valadc2 = 860;
    datosTx.valuef = 83.05f;

    xQueueSend(xQueue, &datosTx, portMAX_DELAY);
  }
}

float Ultrasonic(void)
{
  float distance;

  esp_err_t res = ultrasonic_measure(&sensorU, MAX_DISTANCE_CM, &distance);
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
      distance = MAX_DISTANCE_M;
      break;
    default:
      printf("%s\n", esp_err_to_name(res));
    }
  }

  return distance;
}

// Main
void setup()
{
  Serial.begin(115200);
  Serial.setDebugOutput(true);

  init_gpio();
  init_adc();
  ultrasonic_init(&sensorU);

  setup_wifi();
  vTaskDelay(pdMS_TO_TICKS(1000));

  //****************
  if (!SPIFFS.begin(true))
  {
    Serial.println("Se ha producido un error al montar SPIFFS");
    return;
  }
  //**********************
  // Root CA leer archivo.
  File file2 = SPIFFS.open("/AmazonRootCA1.pem", "r");
  if (!file2)
  {
    Serial.println("No se pudo abrir el archivo para leerlo");
    return;
  }
  Serial.println("Root CA File Content:");
  while (file2.available())
  {
    Read_rootca = file2.readString();
    Serial.println(Read_rootca);
  }
  //*****************************
  // Cert leer archivo
  File file4 = SPIFFS.open("/cd5c02581-certificate.pem.crt", "r");
  if (!file4)
  {
    Serial.println("No se pudo abrir el archivo para leerlo");
    return;
  }
  Serial.println("Cert File Content:");
  while (file4.available())
  {
    Read_cert = file4.readString();
    Serial.println(Read_cert);
  }
  //***************************************
  // Privatekey leer archivo
  File file6 = SPIFFS.open("/cd5c02581-private.pem.key", "r");
  if (!file6)
  {
    Serial.println("No se pudo abrir el archivo para leerlo");
    return;
  }
  Serial.println("privateKey contenido:");
  while (file6.available())
  {
    Read_privatekey = file6.readString();
    Serial.println(Read_privatekey);
  }
  //=====================================================

  char *pRead_rootca;
  pRead_rootca = (char *)malloc(sizeof(char) * (Read_rootca.length() + 1));
  strcpy(pRead_rootca, Read_rootca.c_str());

  char *pRead_cert;
  pRead_cert = (char *)malloc(sizeof(char) * (Read_cert.length() + 1));
  strcpy(pRead_cert, Read_cert.c_str());

  char *pRead_privatekey;
  pRead_privatekey = (char *)malloc(sizeof(char) * (Read_privatekey.length() + 1));
  strcpy(pRead_privatekey, Read_privatekey.c_str());

  Serial.println("================================================================================================");
  Serial.println("Certificados que pasan adjuntan al espClient");
  Serial.println();
  Serial.println("Root CA:");
  Serial.write(pRead_rootca);
  Serial.println("================================================================================================");
  Serial.println();
  Serial.println("Cert:");
  Serial.write(pRead_cert);
  Serial.println("================================================================================================");
  Serial.println();
  Serial.println("privateKey:");
  Serial.write(pRead_privatekey);
  Serial.println("================================================================================================");

  espClient.setCACert(pRead_rootca);
  espClient.setCertificate(pRead_cert);
  espClient.setPrivateKey(pRead_privatekey);

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  //******************************************
  WiFi.macAddress(mac);
  snprintf(mac_Id, sizeof(mac_Id), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.println(mac_Id);
  //****************************************

  //  Create the queue
  xQueue = xQueueCreate(QUEUE_LENGTH, ITEM_SIZE);

  if (xQueue != NULL)
  {
    xTaskCreatePinnedToCore(EntradaDatos, "Entrada Datos ADC y float", 1024 * 4, NULL, 1, NULL, 1);

    xTaskCreatePinnedToCore(SalidaDatos, "Salida Broker MQTT", 1024 * 4, NULL, 1, NULL, 1);

    xTaskCreatePinnedToCore(EnviarAlerta, "Salida Datos Alerta", 1024 * 4, NULL, 1, NULL, 1);
  }
}

// Loop
void loop()
{

  if (!client.connected())
  {
    reconnect();
  }
  client.loop();

  gpio_set_level(LED2, 1);
  vTaskDelay(pdMS_TO_TICKS(1000));
  gpio_set_level(LED2, 0);
  vTaskDelay(pdMS_TO_TICKS(1000));
}