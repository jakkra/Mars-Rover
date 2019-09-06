#include "switch_checker.h"

static uint32_t interval = 1000;
static uint32_t prevUpdateMs;
static RoverMode currentRoverMode = DRIVE_TURN_NORMAL;
static ArmMode currentArmMode = ARM_MODE_MOVE;
static RoverModeChanged* modeChangedCallback;
static ArmModeChanged* armModeCallback;
static uint16_t roverModePin;
static uint16_t armModePin;

void InitSwitchChecker(uint32_t checkIntervalMs, uint16_t roverModeSwitchPin, uint16_t armModeSwitchPin, RoverModeChanged* callback, ArmModeChanged* armCallback) {
    interval = checkIntervalMs;
    modeChangedCallback = callback;
    armModeCallback = armCallback;
    roverModePin = roverModeSwitchPin;
    armModePin = armModeSwitchPin;
}

void SwitchCheckerUpdate() {
    uint16_t signal;
    uint32_t now = millis();
    RoverMode newRoverMode;
    ArmMode newArmMode;

    if (now - prevUpdateMs > interval) {
        signal = pulseIn(roverModePin, HIGH, 100000);
        if (signal < 1400) {
            newRoverMode = DRIVE_TURN_NORMAL;
        } else if (signal > 1600) {
            newRoverMode = ROBOT_ARM;
        } else {
            newRoverMode = DRIVE_TURN_SPIN;
        }

        signal = pulseIn(armModePin, HIGH, 100000);
        if (signal < 1400) {
            newArmMode = ARM_MODE_MOVE;
        } else if (signal > 1600) {
            newArmMode = ARM_MODE_GRIPPER;
        }
        prevUpdateMs = now;
    }
    if (newRoverMode != currentRoverMode) {
        currentRoverMode = newRoverMode;
        modeChangedCallback(newRoverMode);
    }

    if (newArmMode != currentArmMode) {
        currentArmMode = newArmMode;
        armModeCallback(newArmMode);
    }
}