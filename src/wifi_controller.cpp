#include "wifi_controller.h"
#include "rover_config.h"
#include "WiFi.h"
#include "FS.h"
#include "SPIFFS.h"
#include "WebServer.h"
#include <WebSocketsServer.h>
#include "esp_log.h"

#define MAX_REGISTRATED_CALLBACKS 2
#define WEBSOCKET_PORT            81

const char* TAG = "wifi_controller";

static void handle_not_found(void);
static void web_server_handler(void* params);
static void handle_websocket_event(uint8_t num, WStype_t type, uint8_t * payload, size_t length);
static void list_dir(const char* dirname, uint8_t levels);

static uint16_t channel_values[RC_NUM_CHANNELS] = {0};

static const char* wifi_ssid;
static const char* wifi_password;

static IPAddress local_ip(192,168,4,1);
static IPAddress gateway(192,168,1,1);
static IPAddress subnet(255,255,255,0);

static WebServer server(80);
static WebSocketsServer websocket_server = WebSocketsServer(WEBSOCKET_PORT);

static WifiControllerStatusCb* status_callbacks[MAX_REGISTRATED_CALLBACKS];
static uint8_t num_callbacks;
static bool is_initialized = false;

void wifi_controller_init(const char* ssid, const char* password)
{
  BaseType_t status;
  TaskHandle_t xHandle = NULL;
  memset(channel_values, 0, sizeof(channel_values));
  num_callbacks = 0;

  if(!SPIFFS.begin(true)){
    Serial.println("SPIFFS Mount Failed");
    return;
  }
  list_dir("/", 0);
  wifi_ssid = ssid;
  password = password;

  WiFi.softAP(wifi_ssid, wifi_password);
  WiFi.softAPConfig(local_ip, gateway, subnet);

  server.serveStatic("/", SPIFFS, "/index.html", "");
  server.serveStatic("/virtualjoystick.js", SPIFFS, "/virtualjoystick.js", "");
  server.onNotFound(handle_not_found);
  server.begin();
  websocket_server.onEvent(handle_websocket_event);
  websocket_server.begin();

  status = xTaskCreate(web_server_handler, "WebServerHandler", 4096, NULL, tskIDLE_PRIORITY, &xHandle);
  assert(status == pdPASS);
  is_initialized = true;
}

void register_connection_callback(WifiControllerStatusCb* cb) {
  assert(num_callbacks <= MAX_REGISTRATED_CALLBACKS);
  assert(is_initialized);
  status_callbacks[num_callbacks] = cb;
  num_callbacks++;
}

uint16_t wifi_controller_get_val(uint8_t channel)
{
  assert(channel < RC_NUM_CHANNELS);
  return channel_values[channel];
}

static void handle_not_found(void)
{
  server.send(404, "text/plain", "Not found");
}

static void reset_ch_values() {
  for (uint8_t i = 0; i < RC_NUM_CHANNELS; i++) {
    channel_values[i] = config_default_ch_values[i];
  }
}

static void handle_websocket_event(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch (type)
  {
    case WStype_DISCONNECTED:
      printf("Disconnect\n");
      reset_ch_values();
      for (uint8_t i = 0; i < num_callbacks; i++) {
        status_callbacks[i](WIFI_CONTROLLER_DISCONNECTED);
      }
      break;
    case WStype_CONNECTED:
      printf("Connected\n");
      reset_ch_values();
      for (uint8_t i = 0; i < num_callbacks; i++) {
        status_callbacks[i](WIFI_CONTROLLER_CONNECTED);
      }
      break;
    case WStype_TEXT:
      printf("Data: %.*s\n", length, payload);
      break;
    case WStype_BIN:
      if (length >= RC_NUM_CHANNELS * sizeof(uint16_t)) {
        uint16_t* values = (uint16_t*)payload;
        for (uint8_t i = 0; i < RC_NUM_CHANNELS; i++) {
          if (values[i] >= 1000 && values[i] <= 2000) {
            channel_values[i] = values[i];
          } else {
            ESP_LOGE(TAG, "Expected channel values to be in range 1000 - 2000 but was: %d\n", values[i]);
            channel_values[i] = config_default_ch_values[i];
          }
        }
      } else {
        ESP_LOGI(TAG, "Invalid binary length");
        reset_ch_values();
      }
      //printf("%d, %d \t %d, %d \t %d, %d\n", channel_values[0], channel_values[1], channel_values[2],  channel_values[3], channel_values[4], channel_values[5]);
      break;
  default:
    break;
  }
}

static void web_server_handler(void* params)
{
  while(true) {
    websocket_server.loop();
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

static void list_dir(const char* dirname, uint8_t levels) {
    File root = SPIFFS.open(dirname);
    if (!root){
      Serial.println("- failed to open directory");
      return;
    }
    if (!root.isDirectory()) {
      Serial.println(" - not a directory");
      return;
    }

    File file = root.openNextFile();
    while (file) {
      if (file.isDirectory()) {
        Serial.print("  DIR : ");
        Serial.println(file.name());
        if(levels){
          list_dir(file.name(), levels -1);
        }
      } else {
        Serial.print("  FILE: ");
        Serial.print(file.name());
        Serial.print("\tSIZE: ");
        Serial.println(file.size());
      }
      file = root.openNextFile();
    }
}