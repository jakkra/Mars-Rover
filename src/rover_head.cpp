#include <Arduino.h>
#include "rover_head.h"
#include "rover_servo.h"
#include "rover_config.h"

#define HEAD_PITCH_PHYSICAL_MIN_LIMIT_VALUE 1400

void rover_head_init(void) {
  rover_servo_write(SERVO_HEAD_YAW, RC_CENTER, true);
  rover_servo_write(SERVO_HEAD_PITCH, RC_CENTER, true);
}

void rover_head_yaw(uint16_t yaw) {
  uint16_t speed;
  if (yaw < RC_CENTER && yaw > 0) {
    yaw = 2 * RC_CENTER - yaw;
    speed = map(yaw, RC_CENTER, RC_HIGH, SERVO_MIN_SPEED, SERVO_MAX_SPEED);
    rover_servo_move(SERVO_HEAD_YAW, RC_LOW, speed);
  } else if (yaw > RC_CENTER) {
    speed = map(yaw, RC_CENTER, RC_HIGH, SERVO_MIN_SPEED, SERVO_MAX_SPEED);
    rover_servo_move(SERVO_HEAD_YAW, RC_HIGH, speed);
  } else {
    rover_servo_pause(SERVO_HEAD_YAW);
  }
}

void rover_head_pitch(uint16_t pitch) {
  uint16_t speed;
  if (pitch < RC_CENTER && pitch > 0) {
    pitch = 2 * RC_CENTER - pitch;
    speed = map(pitch, RC_CENTER, RC_HIGH, SERVO_MIN_SPEED, SERVO_MAX_SPEED);
    rover_servo_move(SERVO_HEAD_PITCH, HEAD_PITCH_PHYSICAL_MIN_LIMIT_VALUE, speed);
  } else if (pitch > RC_CENTER) {
    speed = map(pitch, RC_CENTER, RC_HIGH, SERVO_MIN_SPEED, SERVO_MAX_SPEED);
    rover_servo_move(SERVO_HEAD_PITCH, RC_HIGH, speed);
  } else {
    rover_servo_pause(SERVO_HEAD_PITCH);
  }
}