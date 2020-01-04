#include <Arduino.h>
#include "arm.h"
#include "rover_config.h"
#include "rover_servo.h"

static RoverServoId axisToServo[ARM_NUM_AXIS] =
{
    [ARM_AXIS_1] = SERVO_ARM_AXIS_1,
    [ARM_AXIS_2] = SERVO_ARM_AXIS_2,
    [ARM_AXIS_3] = SERVO_ARM_AXIS_3,
    [ARM_AXIS_4] = SERVO_ARM_AXIS_4,
    [ARM_AXIS_5] = SERVO_ARM_AXIS_5,
    [ARM_AXIS_6] = SERVO_ARM_AXIS_6,
};

static bool isInitialized = false;

void arm_init()
{

  if (!isInitialized) {
    isInitialized = true;
    // Initial position
    rover_servo_write(axisToServo[ARM_AXIS_1], 1900, true);
    rover_servo_write(axisToServo[ARM_AXIS_2], 1600, true);
    rover_servo_write(axisToServo[ARM_AXIS_3], 1200, true);
    rover_servo_write(axisToServo[ARM_AXIS_4], 1200, true);
    rover_servo_write(axisToServo[ARM_AXIS_5], 1500, true);
    rover_servo_write(axisToServo[ARM_AXIS_6], 1500, true);
  }
}

void arm_move_axis_us(ArmAxis axisNum, uint16_t pos, uint8_t speed)
{
  rover_servo_move(axisToServo[axisNum], pos, speed);
}

void arm_move(ArmAxis axisNum, uint16_t x, uint16_t y, uint16_t z)
{
  // TODO math
}

void arm_pause(ArmAxis axisNum)
{
  rover_servo_pause(axisToServo[axisNum]);
}

void arm_resume(ArmAxis axisNum)
{
  rover_servo_resume(axisToServo[axisNum]);
}

