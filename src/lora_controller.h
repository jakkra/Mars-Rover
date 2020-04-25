#pragma once

#include <Arduino.h>

typedef enum LoraControllerStatus
{
  LORA_CONTROLLER_CONNECTED,
  LORA_CONTROLLER_DISCONNECTED,
} LoraControllerStatus;

typedef void(LoraControllerStatusCb(LoraControllerStatus status));

void lora_controller_init();
void lora_controller_register_connection_callback(LoraControllerStatusCb* cb);
uint16_t lora_controller_get_val(uint8_t channel);