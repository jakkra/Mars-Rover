#include "wifi_controller.h"
#include "rover_config.h"
#include "WiFi.h"
#include "FS.h"
#include "SPIFFS.h"
#include "WebServer.h"
#include "ESPAsyncWebServer.h"
#include "esp_log.h"
#include "assert.h"

#define MAX_REGISTRATED_CALLBACKS 2

static const char* TAG = "wifi_controller";

static void handle_not_found(AsyncWebServerRequest *request);
static void handle_websocket_event(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *payload, size_t length);
static void list_dir(const char* dirname, uint8_t levels);
static void wifiEvent(WiFiEvent_t event, WiFiEventInfo_t info);

static uint16_t channel_values[RC_NUM_CHANNELS] = {0};

static const char* wifi_ssid;
static const char* wifi_password;

static IPAddress local_ip(192,168,4,5);
static IPAddress gateway(192,168,4,1);
static IPAddress subnet(255,255,255,0);

static AsyncWebServer server(80                                                                                                                                                             );
static AsyncWebSocket ws("/ws");

static WifiControllerStatusCb* status_callbacks[MAX_REGISTRATED_CALLBACKS];
static uint8_t num_callbacks;
static bool is_initialized = false;

void wifi_controller_init(const char* ssid, const char* password, WifiControllerMode mode)
{
  memset(channel_values, 0, sizeof(channel_values));
  num_callbacks = 0;

  if(!SPIFFS.begin(true)){
    Serial.println("SPIFFS Mount Failed");
    return;
  }
  list_dir("/", 0);
  wifi_ssid = ssid;
  password = password;

  switch (mode)
  {
  case WIFI_CONTROLLER_STATION:
    WiFi.begin(wifi_ssid, wifi_password);
    WiFi.config(local_ip, gateway, subnet);
    WiFi.setAutoReconnect(true);
    WiFi.onEvent(wifiEvent);
    break;
  case WIFI_CONTROLLER_AP:
    WiFi.softAP(wifi_ssid, wifi_password);
    WiFi.softAPConfig(local_ip, gateway, subnet);
    break;
  default:
    assert(false);
    break;
  }

  ws.onEvent(handle_websocket_event);
  server.addHandler(&ws);

  server.serveStatic("/", SPIFFS, "/index.html", "");
  server.serveStatic("/virtualjoystick.js", SPIFFS, "/virtualjoystick.js", "");
  server.onNotFound(handle_not_found);
  server.begin();

  is_initialized = true;
}

void wifi_controller_register_connection_callback(WifiControllerStatusCb* cb) {
  assert(num_callbacks <= MAX_REGISTRATED_CALLBACKS);
  assert(is_initialized);
  status_callbacks[num_callbacks] = cb;
  num_callbacks++;
}

void wifi_controller_ws_send_bin(uint8_t* data, uint32_t length)
{
    ws.binaryAll(data, length);
}

uint16_t wifi_controller_get_val(uint8_t channel)
{
  assert(channel < RC_NUM_CHANNELS);
  return channel_values[channel];
}

static void handle_not_found(AsyncWebServerRequest *request)
{
  request->send(SPIFFS, "/index.html");
}

static void reset_ch_values() {
  for (uint8_t i = 0; i < RC_NUM_CHANNELS; i++) {
    channel_values[i] = config_default_ch_values[i];
  }
}

static void wifiEvent(WiFiEvent_t event, WiFiEventInfo_t info)
{
  if (event == SYSTEM_EVENT_STA_DISCONNECTED) {
    if (info.disconnected.reason == 6) {
      Serial.println("NOT_AUTHED reconnect");
      WiFi.reconnect();
    }
  }
}

static void handle_websocket_event(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *payload, size_t length)
{
  switch (type)
  {
    case WS_EVT_DISCONNECT:
      printf("Disconnect\n");
      reset_ch_values();
      for (uint8_t i = 0; i < num_callbacks; i++) {
        status_callbacks[i](WIFI_CONTROLLER_DISCONNECTED);
      }
      break;
    case WS_EVT_CONNECT:
      printf("Connected\n");
      reset_ch_values();
      for (uint8_t i = 0; i < num_callbacks; i++) {
        status_callbacks[i](WIFI_CONTROLLER_CONNECTED);
      }
      break;
    case WS_EVT_DATA:
    {
      AwsFrameInfo * info = (AwsFrameInfo*)arg;
      if (!(info->final && info->index == 0 && info->len == length)) {
        break;
      }
      if (info->opcode == WS_BINARY) {
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
      } else {
        printf("Data: %.*s\n", length, payload);
      }
      break;
    }
  default:
    break;
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