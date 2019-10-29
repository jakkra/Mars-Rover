#include "switch_checker.h"
#include "rc_receiver_rmt.h"
#include "wifi_controller.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static void check_switch_channels(void* params);
static void handle_wifi_controller_status(wifi_controller_status status);

static uint32_t interval;
static RoverModeChanged* modeChangedCallback;
static ArmModeChanged* armModeCallback;
static uint16_t roverModeChannel;
static uint16_t armModeChannel;

static bool controller_source_wifi = false;

void init_switch_checker(uint32_t checkIntervalMs, uint16_t roverModeSwitchChannel, uint16_t armModeSwitchChannel, RoverModeChanged* callback, ArmModeChanged* armCallback) {
    BaseType_t status;
    TaskHandle_t xHandle = NULL;
    
    interval = checkIntervalMs;
    modeChangedCallback = callback;
    armModeCallback = armCallback;
    roverModeChannel = roverModeSwitchChannel;
    armModeChannel = armModeSwitchChannel;

    register_connection_callback(&handle_wifi_controller_status);

    status = xTaskCreate(check_switch_channels, "SwitchChecker", 2048, NULL, tskIDLE_PRIORITY, &xHandle);
    assert(status == pdPASS);
}

static void handle_wifi_controller_status(wifi_controller_status status)
{
  if (status == WIFI_CONTROLLER_CONNECTED) {
      controller_source_wifi = true;
  } else {
      controller_source_wifi = false;
  }
}

static void check_switch_channels(void* params)
{
    uint16_t signal;
    RoverMode currentRoverMode = DRIVE_TURN_NORMAL;
    ArmMode currentArmMode = ARM_MODE_MOVE;
    RoverMode newRoverMode = currentRoverMode;
    ArmMode newArmMode = currentArmMode;

    while (true) {
        if (controller_source_wifi) {
            signal = wifi_controller_get_val(roverModeChannel);
        } else {
            signal = rc_receiver_rmt_get_val(roverModeChannel);
        }
        
        if (signal < 1400) {
            newRoverMode = DRIVE_TURN_NORMAL;
        } else if (signal > 1600) {
            newRoverMode = ROBOT_ARM;
        } else {
            newRoverMode = DRIVE_TURN_SPIN;
        }
        signal = rc_receiver_rmt_get_val(armModeChannel);
        if (signal < 1400) {
            newArmMode = ARM_MODE_MOVE;
        } else if (signal > 1600) {
            newArmMode = ARM_MODE_GRIPPER;
        }

        if (newRoverMode != currentRoverMode) {
            currentRoverMode = newRoverMode;
            modeChangedCallback(newRoverMode);
        }

        if (newArmMode != currentArmMode) {
            currentArmMode = newArmMode;
            armModeCallback(newArmMode);
        }
        vTaskDelay(interval * (1000 / configTICK_RATE_HZ)); 
    }
}