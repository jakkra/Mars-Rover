/*
* Code for controlling my Rover. Controlled using generic 6 channel RC Transmitter
* with the receiver connected to an Arduino. 
* 6 motors (one in each wheel) controlled by two ESC's (each side one ESC).
* 4 Servos used for turning, one on each corner wheel.
* 6 Servoes controlling the robot arm of the Rover.
* 
* On the RC Transmitter I use two joysticks for channels 1-4 for, and two switches for channel 5-6.
* Joysticks are used both for steering the Rover and for moving each axis of the arm.
* When switch1 is:
*   -LOW: joysticks are used for driving the rover
*   -MID: Wheels will be set correct angles for the rover to be able to rotate on the spot
*         by running each side of wheels in opposite direction. The direction is descided by the steer joystick.
*   -HIGH: Arm mode, the two joysticks are used for moving axis 1-4 on the arm. Switch2 is used to switch beween moving
*          axis 1-4 and axis 5-6 (the gripper). This will be changed in the future, steering each axis manually is not really a good way, inverse kinematics is TODO.
*/

#include <Arduino.h>
#include "ServoEasing.h"
#include "switch_checker.h"

#define DEBUG

#ifdef DEBUG
#define LOG Serial.print
#if defined(ESP8266) || defined(ESP32)
#define LOGF Serial.printf
#else 
#define LOGF(...) 
#endif
#define LOGLN Serial.println
#else
#define LOG(msg)
#define LOGF(...)
#define LOGLN(msg)
#endif

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

// Input pins connected to each channel on RC Receiver
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
#define RC_SERVO4_CHANNEL_INPUT  RC_CH3_INPUT
#define RC_SERVO5_CHANNEL_INPUT  RC_CH1_INPUT
#define RC_SERVO6_CHANNEL_INPUT  RC_CH2_INPUT

#define RC_STEER_CHANNEL RC_CH1
#define RC_MOTOR_CHANNEL RC_CH2

#define RC_SERVO1_CHANNEL RC_CH1
#define RC_SERVO2_CHANNEL RC_CH2
#define RC_SERVO3_CHANNEL RC_CH4
#define RC_SERVO4_CHANNEL RC_CH3
#define RC_SERVO5_CHANNEL RC_CH1
#define RC_SERVO6_CHANNEL RC_CH2


uint16_t rc_values[RC_NUM_CHANNELS][RC_FILTER_SAMPLES];

MotorDirection motorState = IDLE;
RoverMode currentRoverMode = DRIVE_TURN_NORMAL;
ArmMode currentArmMode = ARM_MODE_MOVE;

ServoEasing Servo1;
ServoEasing Servo2;
ServoEasing Servo3;
ServoEasing Servo4;
ServoEasing Servo5;
ServoEasing Servo6;
ServoEasing motorsLeft;
ServoEasing motorsRight;
ServoEasing frontLeft;
ServoEasing frontRight;
ServoEasing backLeft;
ServoEasing backRight;

int lastMs = 0;
uint8_t rcValueIndex = 0;

/*
* Average out RC_FILTER_SAMPLES samples to get rid of sample errors.
* Remove any invalid signal min is 1000, mid is 1500, high is 2000 us
*/
uint16_t filterSignal(uint16_t* signals) {
  uint32_t signal = 0;
  for (uint8_t i = 0; i < RC_FILTER_SAMPLES; i++) {
    signal += signals[i];
  }
  signal = signal / RC_FILTER_SAMPLES;
  
  if (signal <= 800) return 1500; // If we are unlucky in timing when controller disconnect we might get some random low signal
  if (signal > 800 && signal < 1000) return 1000;
  if (signal > 2000) return 2000;
  if (signal > 1450 && signal < 1550) return 1500;
  return signal;
}

void roverModeChanged(RoverMode mode) {
  LOG("ROVER MODE CHANGED: ");
  LOGLN(mode);
  currentRoverMode = mode;
}

void armModeChanged(ArmMode mode) {
  LOG("ARM MODE CHANGED: ");
  LOGLN(mode);
  currentArmMode = mode;
}

void handleIfControllerDisconnected(uint16_t lastSampledSignal) {
  // Set all RC values to mid position if signal is 0 (controller disconnected)
  if (lastSampledSignal == 0) {
    for (uint8_t channel = 0; channel < RC_NUM_CHANNELS; channel++) {
      for (uint8_t sample = 0; sample < RC_FILTER_SAMPLES; sample++) {
        rc_values[channel][sample] = 1500;
      }
    }
  }
}

void setup() {
  Serial.begin(SERIAL_PORT_SPEED);

  handleIfControllerDisconnected(0);

  pinMode(RC_CH1_INPUT, INPUT);
  pinMode(RC_CH2_INPUT, INPUT);
  pinMode(RC_CH3_INPUT, INPUT);
  pinMode(RC_CH4_INPUT, INPUT);
  pinMode(RC_CH5_INPUT, INPUT);
  pinMode(RC_CH6_INPUT, INPUT);
  InitSwitchChecker(1000, RC_CH5_INPUT, RC_CH6_INPUT, &roverModeChanged, &armModeChanged);

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

  // Steering servos
  frontLeft.attach(A1);
  frontRight.attach(A3);
  backLeft.attach(A0);
  backRight.attach(A2);

  frontLeft.write(90);
  frontRight.write(90);
  backLeft.write(90);
  backRight.write(90);

  // 6 Axis Arm Servos
  Servo1.attach(8);
  Servo2.attach(9);
  Servo3.attach(10);
  Servo4.attach(13);
  Servo5.attach(A4);
  Servo6.attach(A5);

  setEasingTypeForAllServos(EASE_LINEAR);

  Servo1.setSpeed(10);
  Servo2.setSpeed(10);
  Servo3.setSpeed(10);
  Servo4.setSpeed(10);
  Servo5.setSpeed(10);
  Servo6.setSpeed(10);

  // Arm start position
  Servo1.write(165);
  Servo2.write(90);
  Servo3.write(0);
  Servo4.write(50);
  Servo5.write(90);
  Servo6.write(90);

  lastMs = millis();
  LOGLN("Ready");
}

void handleRobotArmServo(ServoEasing* servo, uint16_t channel) {
  uint16_t speed;
  uint16_t signal = filterSignal(rc_values[channel]);
  if (signal < 1500 && signal > 0) {
    speed = map(signal, 1000, 1500, 0, MAX_SERVO_ANGLE);
    speed = max(MAX_SERVO_ANGLE - speed, 1);
    servo->startEaseTo(MIN_SERVO_ANGLE, speed);
  } else if (signal > 1500) {
    speed = map(signal, 1500, 2000, 0, MAX_SERVO_ANGLE);
    speed = max(speed, 1);
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
  LOGLN(signal);
  switch (currentRoverMode) {
    case DRIVE_TURN_NORMAL:
        if (signal < 1500 && signal > 0) { // Backward        
          // When going from forward/idle to reverse, motors (ESC) need to be set in reverse mode
          if (motorState == IDLE) {
              setMotorInReverseMode(motorsLeft);
              setMotorInReverseMode(motorsRight);
              motorState = BACKWARD;
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

        if (signal < 1500 && signal > 0) { // Spin Anticlockwise
          if (motorState == IDLE) {
            setMotorInReverseMode(motorsLeft);
            motorState = FORWARD;
          }
          motorsRight.writeMicroseconds(1500 + diff);
          motorsLeft.writeMicroseconds(1500 - diff);
        } else if (signal > 1500) {
          if (motorState == IDLE) {
            setMotorInReverseMode(motorsRight);
            motorState = FORWARD;
          }
          motorsRight.writeMicroseconds(1500 - diff); // Spin Clockwise
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
  uint16_t diff = abs((int16_t)1500 - (int16_t)signal);
  if (signal < 1500 && signal > 0) { // Left turn
      frontLeft.writeMicrosecondsOrUnits(1500 - diff);
      frontRight.writeMicrosecondsOrUnits(1500 - diff);
      backLeft.writeMicrosecondsOrUnits(1500 + diff);
      backRight.writeMicrosecondsOrUnits(1500 + diff);
  } else if (signal > 1500) { // Right turn
      frontLeft.writeMicrosecondsOrUnits(1500 + diff);
      frontRight.writeMicrosecondsOrUnits(1500 + diff);
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
  lastMs = time;
  switch (currentRoverMode) {
    case DRIVE_TURN_NORMAL:
    case DRIVE_TURN_SPIN:
    {
      uint16_t signal;

      rc_values[RC_STEER_CHANNEL][rcValueIndex] = pulseIn(RC_STEER_CHANNEL_INPUT, HIGH, 100000);
      rc_values[RC_MOTOR_CHANNEL][rcValueIndex] = pulseIn(RC_MOTOR_CHANNEL_INPUT, HIGH, 100000);
      handleIfControllerDisconnected(rc_values[RC_STEER_CHANNEL][rcValueIndex]);

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
      if (currentArmMode == ARM_MODE_MOVE) {
        rc_values[RC_SERVO1_CHANNEL][rcValueIndex] = pulseIn(RC_SERVO1_CHANNEL_INPUT, HIGH, 100000);
        rc_values[RC_SERVO2_CHANNEL][rcValueIndex] = pulseIn(RC_SERVO2_CHANNEL_INPUT, HIGH, 100000);
        rc_values[RC_SERVO3_CHANNEL][rcValueIndex] = pulseIn(RC_SERVO3_CHANNEL_INPUT, HIGH, 100000);
        rc_values[RC_SERVO4_CHANNEL][rcValueIndex] = pulseIn(RC_SERVO4_CHANNEL_INPUT, HIGH, 100000);
        handleIfControllerDisconnected(rc_values[RC_SERVO1_CHANNEL][rcValueIndex]);

        handleRobotArmServo(&Servo1, RC_SERVO1_CHANNEL);
        handleRobotArmServo(&Servo2, RC_SERVO2_CHANNEL);
        handleRobotArmServo(&Servo3, RC_SERVO3_CHANNEL);
        handleRobotArmServo(&Servo4, RC_SERVO4_CHANNEL);
      } else if (currentArmMode == ARM_MODE_GRIPPER) {
        rc_values[RC_SERVO5_CHANNEL][rcValueIndex] = pulseIn(RC_SERVO5_CHANNEL_INPUT, HIGH, 100000);
        rc_values[RC_SERVO6_CHANNEL][rcValueIndex] = pulseIn(RC_SERVO6_CHANNEL_INPUT, HIGH, 100000);
        handleIfControllerDisconnected(rc_values[RC_SERVO5_CHANNEL][rcValueIndex]);

        handleRobotArmServo(&Servo5, RC_SERVO5_CHANNEL);
        handleRobotArmServo(&Servo6, RC_SERVO6_CHANNEL);
      }
      break;
  }

  rcValueIndex = (rcValueIndex + 1) % RC_FILTER_SAMPLES;
}