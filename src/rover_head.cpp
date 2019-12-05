#include <Arduino.h>
#include "rover_head.h"
#include "rover_servo.h"
#include "rover_config.h"

#define HEAD_PITCH_DEFAULT_POS_OFFSET 75

void rover_head_init(void) {
  rover_servo_write(SERVO_HEAD_YAW, RC_CENTER, true);
  rover_servo_write(SERVO_HEAD_PITCH, RC_CENTER, true);
}

void rover_head_yaw(uint16_t yaw) {
  rover_servo_write(SERVO_HEAD_YAW, yaw, true);
}

void rover_head_pitch(uint16_t pitch) {
  if (pitch >= 1400 - HEAD_PITCH_DEFAULT_POS_OFFSET) { // Physical limitations of head
    rover_servo_write(SERVO_HEAD_PITCH, pitch + HEAD_PITCH_DEFAULT_POS_OFFSET, true);
  }
}