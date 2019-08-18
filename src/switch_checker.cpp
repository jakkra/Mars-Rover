#include "switch_checker.h"

uint32_t interval = 1000;
uint32_t prevUpdateMs;
RoverMode currentMode = DRIVE_TURN_NORMAL;
RoverModeChanged* modeChangedCallback;
uint16_t switchPin;

void InitSwitchChecker(uint32_t checkIntervalMs, uint16_t pin, RoverModeChanged* callback) {
    interval = checkIntervalMs;
    modeChangedCallback = callback;
    switchPin = pin;
}

void SwitchCheckerUpdate() {
    uint16_t signal;
    uint32_t now = millis();
    RoverMode newMode;
    if (now - prevUpdateMs > interval) {
        signal = pulseIn(switchPin, HIGH, 100000);
        if (signal < 1400) {
            newMode = DRIVE_TURN_NORMAL;
        } else if (signal > 1600) {
            newMode = ROBOT_ARM;
        } else {
            newMode = DRIVE_TURN_SPIN;
        }
        prevUpdateMs = now;
    }
    if (newMode != currentMode) {
        currentMode = newMode;
        modeChangedCallback(newMode);
    }
}