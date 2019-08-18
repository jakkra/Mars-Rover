#include <Arduino.h>

typedef enum RoverMode {
    DRIVE_TURN_NORMAL,
    DRIVE_TURN_SPIN,
    ROBOT_ARM,
} RoverMode;

typedef void (RoverModeChanged)(RoverMode mode);

void InitSwitchChecker(uint32_t checkIntervalMs, uint16_t pin, RoverModeChanged* callback);
void SwitchCheckerUpdate();