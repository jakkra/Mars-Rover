#include <Arduino.h>
#include <ESP32Servo.h>

#define ARM_MAX_SPEED 10
#define ARM_MIN_SPEED 1

typedef enum ArmAxis {
  ARM_AXIS_1,
  ARM_AXIS_2,
  ARM_AXIS_3,
  ARM_AXIS_4,
  ARM_AXIS_5,
  ARM_AXIS_6,
} ArmAxis;

void arm_init(Servo* servos, uint8_t num_servos);

void arm_move_axis_us(ArmAxis axisNum, uint16_t pos, uint8_t speed);

void arm_move(ArmAxis axisNum, uint16_t x, uint16_t y, uint16_t z);

void arm_pause(ArmAxis axisNum);

void arm_resume(ArmAxis axisNum);
