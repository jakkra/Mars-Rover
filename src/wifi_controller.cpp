#include "wifi_controller.h"
#include "rover_config.h"
#include "WiFi.h"
#include "WebServer.h"
#include <WebSocketsServer.h>
#include "esp_log.h"

const char* TAG = "wifi_controller";

static void handle_on_root(void);
static void handle_not_found(void);
static void web_server_handler(void* params);
static void handle_websocket_event(uint8_t num, WStype_t type, uint8_t * payload, size_t length);

static uint16_t channel_values[RC_NUM_CHANNELS] = {0};

static const char* wifi_ssid;
static const char* wifi_password;
static wifi_controller_status_cb* status_cb = NULL;

static IPAddress local_ip(192,168,4,1);
static IPAddress gateway(192,168,1,1);
static IPAddress subnet(255,255,255,0);

static WebServer server(80);
static WebSocketsServer websocket_server = WebSocketsServer(81);

void wifi_controller_init(const char* ssid, const char* password, wifi_controller_status_cb* cb)
{
  BaseType_t status;
  TaskHandle_t xHandle = NULL;
  memset(channel_values, 0, sizeof(channel_values));

  wifi_ssid = ssid;
  password = password;

  WiFi.softAP(wifi_ssid, wifi_password);
  WiFi.softAPConfig(local_ip, gateway, subnet);

  status_cb = cb;

  server.on("/", HTTP_GET , handle_on_root);
  server.onNotFound(handle_not_found);
  server.begin();
  websocket_server.onEvent(handle_websocket_event);
  websocket_server.begin();

  status = xTaskCreate(web_server_handler, "WebServerHandler", 4096, NULL, tskIDLE_PRIORITY, &xHandle);
  assert(status == pdPASS);
}

uint16_t wifi_controller_get_val(uint8_t channel)
{
  assert(channel < RC_NUM_CHANNELS);
  return channel_values[channel];
}

static void handle_on_root(void)
{
  // TODO create and send JS which connects over websocket to port 81
  server.send(200, "text/plain", "Ready");
}

static void handle_not_found(void)
{
  server.send(404, "text/plain", "Not found");
}

static void handle_websocket_event(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch (type)
  {
    case WStype_DISCONNECTED:
      assert(status_cb != NULL);
      memset(channel_values, 0, sizeof(channel_values));
      status_cb(WIFI_CONTROLLER_DISCONNECTED);
      break;
    case WStype_CONNECTED:
      assert(status_cb != NULL);
      memset(channel_values, 0, sizeof(channel_values));
      status_cb(WIFI_CONTROLLER_CONNECTED);
      break;
    case WStype_TEXT:
      break;
    case WStype_BIN:
      if (length >= RC_NUM_CHANNELS * sizeof(uint16_t)) {
        uint16_t* values = (uint16_t*)payload;
        for (uint8_t i = 0; i < RC_NUM_CHANNELS; i++) {
          if (values[i] >= 1000 && values[i] <= 2000) {
            channel_values[i] = values[i];
          } else {
            ESP_LOGE(TAG, "Expected channel values to be in range 1000 - 2000 but was: %d\n", values[i]);
            channel_values[i] = 1500;
          }
        }
      } else {
        ESP_LOGI(TAG, "Invalid binary length");
      }
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