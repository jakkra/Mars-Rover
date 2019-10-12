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


void init_switch_checker(uint32_t checkIntervalMs, uint16_t roverModeSwitchChannel, uint16_t armModeSwitchChannel, RoverModeChanged* callback, ArmModeChanged* armCallback);