#pragma once

#include <Arduino.h>

#define SERVO_MAX_SPEED 10
#define SERVO_MIN_SPEED 1

// Order of servos in PWM Expander
typedef enum RoverServoId {
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
} RoverServoId;

void rover_servo_init(xSemaphoreHandle i2cSemHandle);
void rover_servo_write(RoverServoId servoId, uint16_t us, bool full_range = false);

void rover_servo_move(RoverServoId servoId, uint16_t pos, uint8_t speed);
void rover_servo_pause(RoverServoId servoId);
void rover_servo_resume(RoverServoId servoId);
