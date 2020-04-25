#include "switch_checker.h"
#include "rc_receiver_rmt.h"
#include "wifi_controller.h"
#include "lora_controller.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rover_config.h"

typedef enum ControllerSource {
  SOURCE_RC,
  SOURCE_WIFI,
  SOURCE_LORA
} ControllerSource;

static void check_switch_channels(void* params);
static void handle_wifi_controller_status(WifiControllerStatus status);
static void handle_lora_controller_status(LoraControllerStatus status);
static void update_controller_source(void);
static uint16_t get_channel_value(uint16_t channel);

static uint32_t interval;
static RoverModeChanged* mode_changed_callback;
static ArmModeChanged* arm_mode_callback;
static uint16_t rover_mode_channel;
static uint16_t arm_mode_channel;

static ControllerSource controller_source = SOURCE_RC;
static WifiControllerStatus wifi_state = WIFI_CONTROLLER_DISCONNECTED;
static LoraControllerStatus lora_state = LORA_CONTROLLER_DISCONNECTED;

void init_switch_checker(uint32_t check_interval_ms, uint16_t rover_mode_switch_channel, uint16_t arm_mode_switch_channel, RoverModeChanged* callback, ArmModeChanged* arm_callback, bool wifi_enabled) {
  BaseType_t status;
  TaskHandle_t xHandle = NULL;
  
  interval = check_interval_ms;
  mode_changed_callback = callback;
  arm_mode_callback = arm_callback;
  rover_mode_channel = rover_mode_switch_channel;
  arm_mode_channel = arm_mode_switch_channel;

  if (wifi_enabled) {
    wifi_controller_register_connection_callback(&handle_wifi_controller_status);
  }
  lora_controller_register_connection_callback(&handle_lora_controller_status);

  status = xTaskCreate(check_switch_channels, "SwitchChecker", 2048, NULL, tskIDLE_PRIORITY, &xHandle);
  assert(status == pdPASS);
}

static void handle_wifi_controller_status(WifiControllerStatus status)
{
  wifi_state = status;
  update_controller_source();
}

static void handle_lora_controller_status(LoraControllerStatus status)
{
  lora_state = status;
  update_controller_source();
}

static void update_controller_source(void)
{
  if (lora_state == LORA_CONTROLLER_CONNECTED) {
    controller_source = SOURCE_LORA;
  } else if (wifi_state == WIFI_CONTROLLER_CONNECTED) {
    controller_source = SOURCE_WIFI;
  } else {
    controller_source = SOURCE_RC;
  }
}

static void check_switch_channels(void* params)
{
  uint16_t signal;
  RoverMode current_rover_mode = DRIVE_TURN_NORMAL;
  ArmMode current_arm_mode = ARM_MODE_MOVE;
  RoverMode new_rover_mode = current_rover_mode;
  ArmMode new_arm_mode = current_arm_mode;

  while (true) {
    
    signal = get_channel_value(rover_mode_channel);
    
    if (signal < RC_CENTER - 100) {
      new_rover_mode = DRIVE_TURN_NORMAL;
    } else if (signal > RC_CENTER + 100) {
      new_rover_mode = ROBOT_ARM;
    } else {
      new_rover_mode = DRIVE_TURN_SPIN;
    }
    
    signal = get_channel_value(arm_mode_channel);

    if (signal < RC_CENTER - 100) {
      new_arm_mode = ARM_MODE_MOVE;
    } else if (signal > RC_CENTER + 100) {
      new_arm_mode = ARM_MODE_GRIPPER;
    }

    if (new_rover_mode != current_rover_mode) {
      current_rover_mode = new_rover_mode;
      mode_changed_callback(new_rover_mode);
    }

    if (new_arm_mode != current_arm_mode) {
      current_arm_mode = new_arm_mode;
      arm_mode_callback(new_arm_mode);
    }
    vTaskDelay(interval * (1000 / configTICK_RATE_HZ)); 
  }
}

static uint16_t get_channel_value(uint16_t channel)
{
  uint16_t value;
  switch (controller_source) {
    case SOURCE_RC:
      value = rc_receiver_rmt_get_val(channel);
      break;
    case SOURCE_WIFI:
      value = wifi_controller_get_val(channel);
      break;
    case SOURCE_LORA:
      value = lora_controller_get_val(channel);
      break;
    default:
      assert(false);
      break;
  }
  return value;
}