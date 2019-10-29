#include <Arduino.h>

typedef enum wifi_controller_status
{
  WIFI_CONTROLLER_CONNECTED,
  WIFI_CONTROLLER_DISCONNECTED,
  WIFI_CONTROLLER_ERROR,
} wifi_controller_status;

typedef void(wifi_controller_status_cb(wifi_controller_status status));

void wifi_controller_init(const char* ssid, const char* password);
void register_connection_callback(wifi_controller_status_cb* cb);
void wifi_controller_deinit(void);
uint16_t wifi_controller_get_val(uint8_t channel);