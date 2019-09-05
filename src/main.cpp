#include <Arduino.h>
#include "ServoEasing.h"

#include "switch_checker.h"

typedef struct {
  uint8_t forward_value;
  uint8_t endstop_min;
  uint8_t endstop_max;
  Servo servo;
} calibrated_servo;

enum MotorDirection {
  IDLE,
  FORWARD,
  BACKWARD
};

#define SERIAL_PORT_SPEED 9600

#define RC_NUM_CHANNELS   6
#define RC_FILTER_SAMPLES 3

#define SERVO_MOVE_SPEED 60
#define MIN_SERVO_ANGLE 20
#define MAX_SERVO_ANGLE 160

#define SERVO_STEER_SPEED 180

#define RC_CH1  0
#define RC_CH2  1
#define RC_CH3  2
#define RC_CH4  3
#define RC_CH5  4
#define RC_CH6  5

#define RC_CH1_INPUT  2
#define RC_CH2_INPUT  3
#define RC_CH3_INPUT  4
#define RC_CH4_INPUT  5
#define RC_CH5_INPUT  6
#define RC_CH6_INPUT  7

#define RC_STEER_CHANNEL_INPUT  RC_CH1_INPUT
#define RC_MOTOR_CHANNEL_INPUT  RC_CH2_INPUT

#define RC_SERVO1_CHANNEL_INPUT  RC_CH1_INPUT
#define RC_SERVO2_CHANNEL_INPUT  RC_CH2_INPUT
#define RC_SERVO3_CHANNEL_INPUT  RC_CH4_INPUT

#define RC_STEER_CHANNEL RC_CH1
#define RC_MOTOR_CHANNEL RC_CH2

#define RC_SERVO1_CHANNEL RC_CH1
#define RC_SERVO2_CHANNEL RC_CH2
#define RC_SERVO3_CHANNEL RC_CH4


uint16_t rc_values[RC_NUM_CHANNELS][RC_FILTER_SAMPLES];

MotorDirection motorState = IDLE;
RoverMode currentRoverMode = DRIVE_TURN_NORMAL;

ServoEasing Servo1;
ServoEasing Servo2;
ServoEasing Servo3;
ServoEasing motorsLeft;
ServoEasing motorsRight;
ServoEasing frontLeft;
ServoEasing frontRight;
ServoEasing backLeft;
ServoEasing backRight;

int lastMs = 0;
uint8_t rcValueIndex = 0;

uint16_t filterSignal(uint16_t* signals) {
  uint32_t signal = 0;
  for (uint8_t i = 0; i < RC_FILTER_SAMPLES; i++) {
    signal += signals[i];
  }
  signal = signal / RC_FILTER_SAMPLES;
  
  if (signal == 0) return 0;
  if (signal < 1000) return 1000;
  if (signal > 2000) return 2000;
  if (signal > 1450 && signal < 1550) return 1500;
  return signal;
}

void modeChanged(RoverMode mode) {
  Serial.print("MODE CHANGED: ");
  Serial.println(mode);
  currentRoverMode = mode;
}

void resetRcValues(void) {
  // Set all RC values to mid position
  for (uint8_t channel = 0; channel < RC_NUM_CHANNELS; channel++) {
    for (uint8_t sample = 0; sample < RC_FILTER_SAMPLES; sample++) {
      rc_values[channel][sample] = 1500;
    }
  }
}

void setup() {
  Serial.begin(SERIAL_PORT_SPEED);

  resetRcValues();

  pinMode(RC_CH1_INPUT, INPUT);
  pinMode(RC_CH2_INPUT, INPUT);
  pinMode(RC_CH3_INPUT, INPUT);
  pinMode(RC_CH4_INPUT, INPUT);
  pinMode(RC_CH5_INPUT, INPUT);
  pinMode(RC_CH6_INPUT, INPUT);
  InitSwitchChecker(1000, RC_CH5_INPUT, &modeChanged);
  
  motorsLeft.attach(12);
  motorsLeft.writeMicroseconds(1500);
  motorsRight.attach(11);
  motorsRight.writeMicroseconds(1500);
  /*
  A0 = backLeft
  A1 = frontLeft
  A2 = backRight 
  A3 = frontRight
  */
  frontLeft.attach(A1);
  frontRight.attach(A3);
  backLeft.attach(A0);
  backRight.attach(A2);

  frontLeft.write(90);
  frontRight.write(90);
  backLeft.write(90);
  backRight.write(90);

  Servo1.attach(8);
  Servo2.attach(9);
  Servo3.attach(10);
  setEasingTypeForAllServos(EASE_LINEAR);
  Servo1.setSpeed(10);
  Servo2.setSpeed(10);
  Servo3.setSpeed(10);
  Servo1.setEaseTo(90);
  Servo2.setEaseTo(90);
  Servo3.startEaseTo(90);
  //while (Servo1.isMoving() || Servo2.isMoving() || Servo3.isMoving());
  Serial.println("Ready");
  lastMs = millis();
}

void handleRobotArmServo21(ServoEasing* servo, uint16_t channel) {
  uint16_t speed;
  uint16_t signal = filterSignal(rc_values[channel]);
  if (signal < 1500 && signal > 0) {
    //Serial.print("CH:"); Serial.print(channel); Serial.print(" "); Serial.print(rc_values[channel]); Serial.print("\t");
    speed = map(signal, 1000, 1500, 0, MAX_SERVO_ANGLE);
    speed = max(MAX_SERVO_ANGLE - speed, 1);
    //Serial.print("Signal: ");
    //Serial.print(signal);
    //Serial.print("Speed: ");
    //Serial.println(speed);
    servo->startEaseTo(MIN_SERVO_ANGLE, speed);
  } else if (signal > 1500) {
    //Serial.print("CH:"); Serial.print(channel); Serial.print(" "); Serial.print(rc_values[channel]); Serial.print("\t");
    speed = map(signal, 1500, 2000, 0, MAX_SERVO_ANGLE);
    speed = max(speed, 1);
    //Serial.print("Signal: ");
    //Serial.print(signal);
    //Serial.print("Speed: ");
    //Serial.println(speed);
    servo->startEaseTo(MAX_SERVO_ANGLE, speed);
  } else {
    servo->mServoMoves = false;
    if (!isOneServoMoving()) {
      stopAllServos();
    }
  }
}

void setMotorInReverseMode(ServoEasing motor) {
  motor.writeMicroseconds(1400);
  delay(100);
  motor.writeMicroseconds(1500);
  delay(100);
}

void handleMoveMotors(uint16_t signal) {
  Serial.println(signal);

  switch (currentRoverMode) {
    case DRIVE_TURN_NORMAL:
        if (signal < 1500 && signal > 0) { // Backward        
          switch (motorState) {
            case IDLE:
              Serial.println("Was IDLE");
              setMotorInReverseMode(motorsLeft);
              setMotorInReverseMode(motorsRight);
              motorState = BACKWARD;
              break;
            case FORWARD:
              break;
            case BACKWARD:
              break;
          }
        
      } else if (signal > 1500) {
        motorState = FORWARD;
      } else {
        motorState = IDLE;
      }
      motorsLeft.writeMicroseconds(signal);
      motorsRight.writeMicroseconds(signal);
      break;
    case DRIVE_TURN_SPIN:
      {
        uint16_t diff = abs((int16_t)1500 - (int16_t)signal);
        Serial.print(signal);
        Serial.print(" ");
        Serial.println(diff);

        if (signal < 1500 && signal > 0) { // Anticlockwise
          if (motorState == IDLE) {
            Serial.println("Was Idle");
            setMotorInReverseMode(motorsLeft);
            motorState = FORWARD;
          }
          motorsRight.writeMicroseconds(1500 + diff);
          motorsLeft.writeMicroseconds(1500 - diff);
        } else if (signal > 1500) {
          if (motorState == IDLE) {
            Serial.println("Was Idle");
            setMotorInReverseMode(motorsRight);
            motorState = FORWARD;
          }
          motorsRight.writeMicroseconds(1500 - diff); // Clockwise
          motorsLeft.writeMicroseconds(1500 + diff);
        } else {
          motorState = IDLE;
          motorsLeft.writeMicroseconds(signal);
          motorsRight.writeMicroseconds(signal);
        }
        break;
      }
    default:
      break;
  }

}

void steerNormal(uint16_t signal) {
  if (signal < 1500 && signal > 0) { // Left turn
      uint16_t diff = 1500 - signal;
      frontLeft.writeMicrosecondsOrUnits(signal);
      frontRight.writeMicrosecondsOrUnits(signal);
      backLeft.writeMicrosecondsOrUnits(1500 + diff);
      backRight.writeMicrosecondsOrUnits(1500 + diff);
  } else if (signal > 1500) { // Right turn
      uint16_t diff = signal - 1500;
      frontLeft.writeMicrosecondsOrUnits(signal);
      frontRight.writeMicrosecondsOrUnits(signal);
      backLeft.writeMicrosecondsOrUnits(1500 - diff);
      backRight.writeMicrosecondsOrUnits(1500 - diff);
  } else {
    frontLeft.writeMicrosecondsOrUnits(1500);
    frontRight.writeMicrosecondsOrUnits(1500);
    backLeft.writeMicrosecondsOrUnits(1500);
    backRight.writeMicrosecondsOrUnits(1500);
  }
}

void steerSpin(uint16_t signal) {
  frontLeft.writeMicrosecondsOrUnits(2000);
  frontRight.writeMicrosecondsOrUnits(1000);
  backLeft.writeMicrosecondsOrUnits(1000);
  backRight.writeMicrosecondsOrUnits(2000);
}

void handleSteer(void) {
  uint16_t signal = filterSignal(rc_values[RC_STEER_CHANNEL]);
  switch (currentRoverMode) {
    case DRIVE_TURN_NORMAL:
      steerNormal(signal);
      break;
    case DRIVE_TURN_SPIN:
      steerSpin(signal);
      break;
    default:
      break;
    }
  }

void loop() {
  SwitchCheckerUpdate();
  int time = millis();
  //Serial.print("Loop: ");
  //Serial.println(time - lastMs);
  lastMs = time;
  switch (currentRoverMode) {
    case DRIVE_TURN_NORMAL:
    case DRIVE_TURN_SPIN:
    {
      uint16_t signal;

      rc_values[RC_STEER_CHANNEL][rcValueIndex] = pulseIn(RC_STEER_CHANNEL_INPUT, HIGH, 100000);
      rc_values[RC_MOTOR_CHANNEL][rcValueIndex] = pulseIn(RC_MOTOR_CHANNEL_INPUT, HIGH, 100000);

      // 0 if controller disconnected 
      if (rc_values[RC_STEER_CHANNEL][rcValueIndex] == 0) {
        resetRcValues();
      }

      if (currentRoverMode == DRIVE_TURN_NORMAL) {
        signal = filterSignal(rc_values[RC_MOTOR_CHANNEL]);
      } else if (currentRoverMode == DRIVE_TURN_SPIN) {
        signal = filterSignal(rc_values[RC_STEER_CHANNEL]);
      }
      handleMoveMotors(signal);
      handleSteer();
      break;
    }
    case ROBOT_ARM:
      rc_values[RC_SERVO1_CHANNEL][rcValueIndex] = pulseIn(RC_SERVO1_CHANNEL_INPUT, HIGH, 100000);
      rc_values[RC_SERVO2_CHANNEL][rcValueIndex] = pulseIn(RC_SERVO2_CHANNEL_INPUT, HIGH, 100000);
      rc_values[RC_SERVO3_CHANNEL][rcValueIndex] = pulseIn(RC_SERVO3_CHANNEL_INPUT, HIGH, 100000);

      if (rc_values[RC_SERVO1_CHANNEL][rcValueIndex] == 0) {
        resetRcValues();
      }

      handleRobotArmServo21(&Servo1, RC_CH1);
      handleRobotArmServo21(&Servo2, RC_CH2);
      handleRobotArmServo21(&Servo3, RC_CH4);
      break;
  }

  rcValueIndex = (rcValueIndex + 1) % RC_FILTER_SAMPLES;
}