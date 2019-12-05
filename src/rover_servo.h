#pragma once

#include <Arduino.h>

// Order of servos in PWM Expander
typedef enum RoverServo {
  SERVO_ARM_AXIS_1,
  SERVO_ARM_AXIS_2,
  SERVO_ARM_AXIS_3,
  SERVO_ARM_AXIS_4,
  SERVO_ARM_AXIS_5,
  SERVO_ARM_AXIS_6,
  SERVO_FRONT_RIGHT,
  SERVO_BACK_LEFT,
  SERVO_FRONT_LEFT,
  SERVO_BACK_RIGHT,
  SERVO_UNUSED,
  SERVO_HEAD_YAW,
  SERVO_HEAD_PITCH,
  SERVO_LAST,
} RoverServo;

void rover_servo_init();
void rover_servo_write(RoverServo axis, uint16_t us, bool full_range = false);
