#include <Arduino.h>

typedef enum RoverMode {
    DRIVE_TURN_NORMAL,
    DRIVE_TURN_SPIN,
    ROBOT_ARM,
} RoverMode;

typedef enum ArmMode {
    ARM_MODE_MOVE,
    ARM_MODE_GRIPPER,
} ArmMode;

typedef void (RoverModeChanged)(RoverMode mode);
typedef void (ArmModeChanged)(ArmMode mode);


void InitSwitchChecker(uint32_t checkIntervalMs, uint16_t roverModeSwitchPin, uint16_t armModeSwitchPin, RoverModeChanged* callback, ArmModeChanged* armCallback);
void SwitchCheckerUpdate();