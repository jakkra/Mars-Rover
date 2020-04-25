#pragma once

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


void init_switch_checker(uint32_t check_interval_ms, uint16_t rover_mode_switch_channel, uint16_t arm_mode_switch_channel, RoverModeChanged* callback, ArmModeChanged* arm_callback, bool wifi_enabled);