#include "switch_checker.h"
#include "rc_receiver_rmt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static void check_switch_channels(void* params);

static uint32_t interval;
static RoverModeChanged* modeChangedCallback;
static ArmModeChanged* armModeCallback;
static uint16_t roverModeChannel;
static uint16_t armModeChannel;

void init_switch_checker(uint32_t checkIntervalMs, uint16_t roverModeSwitchChannel, uint16_t armModeSwitchChannel, RoverModeChanged* callback, ArmModeChanged* armCallback) {
    BaseType_t status;
    TaskHandle_t xHandle = NULL;
    
    interval = checkIntervalMs;
    modeChangedCallback = callback;
    armModeCallback = armCallback;
    roverModeChannel = roverModeSwitchChannel;
    armModeChannel = armModeSwitchChannel;

    status = xTaskCreate(check_switch_channels, "SwitchChecker", 2048, NULL, tskIDLE_PRIORITY, &xHandle);
    assert(status == pdPASS);
}

static void check_switch_channels(void* params)
{
    uint16_t signal;
    RoverMode currentRoverMode = DRIVE_TURN_NORMAL;
    ArmMode currentArmMode = ARM_MODE_MOVE;
    RoverMode newRoverMode = currentRoverMode;
    ArmMode newArmMode = currentArmMode;

    while (true) {
        signal = rc_receiver_rmt_get_val(roverModeChannel);
        
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