#pragma once

#include <Arduino.h>

typedef enum WifiControllerStatus
{
  WIFI_CONTROLLER_CONNECTED,
  WIFI_CONTROLLER_DISCONNECTED,
  WIFI_CONTROLLER_ERROR,
} WifiControllerStatus;

typedef void(WifiControllerStatusCb(WifiControllerStatus status));

void wifi_controller_init(const char* ssid, const char* password);
void wifi_controller_register_connection_callback(WifiControllerStatusCb* cb);
void wifi_controller_ws_send_bin(uint8_t* data, uint32_t length);
void wifi_controller_deinit(void);
uint16_t wifi_controller_get_val(uint8_t channel);