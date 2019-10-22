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
#include "rover_config.h"
#include "esp_log.h"
#include <ESP32Servo.h>
#include "Wire.h"
#include "rc_receiver_rmt.h"
#include "switch_checker.h"
#include "arm.h"
#include "gyro_accel_sensor.h"
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

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

enum MotorDirection {
  MOTOR_IDLE,
  MOTOR_FORWARD,
  MOTOR_BACKWARD
};

uint16_t rc_values[RC_NUM_CHANNELS][RC_FILTER_SAMPLES];
uint8_t rcValueIndex = 0;

MotorDirection motorState = MOTOR_IDLE;
RoverMode currentRoverMode = DRIVE_TURN_NORMAL;
ArmMode currentArmMode = ARM_MODE_MOVE;

static Servo arm_servos[ARM_NUM_AXIS];

Servo motorsLeft;
Servo motorsRight;

Servo frontLeft;
Servo frontRight;
Servo backLeft;
Servo backRight;


uint16_t filter_signal(uint16_t* signals) {
  uint32_t signal = 0;
  for (uint8_t i = 0; i < RC_FILTER_SAMPLES; i++) {
    signal += signals[i];
  }
  signal = signal / RC_FILTER_SAMPLES;
  
  if (signal <= 800) return 1500; // If we are unlucky in timing when controller disconnect we might get some random low signal
  if (signal > 800 && signal < 1010) return 1000;
  if (signal > 1990) return 2000;
  if (signal > 1480 && signal < 1520) return 1500;
  return signal;;
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

#define INTERRUPT_PIN 34

const signed char orientationDefault[9] = { 0, 1, 0, 0, 0, 1, 1, 0, 0 };

void setup() {
    Serial.begin(SERIAL_PORT_SPEED);
    ESP_LOGI("", "START");
    Wire.begin(22, 27);
    Wire.setClock(400000);
    gyro_accel_init(true);
    return;

    handleIfControllerDisconnected(0);
    rc_receiver_rmt_init();
    init_switch_checker(1000, RC_ARM_MODE_ROVER_CHANNEL, RC_ROVER_MODE_ROVER_CHANNEL, &roverModeChanged, &armModeChanged);

    motorsLeft.attach(12);
    motorsLeft.writeMicroseconds(1500);
    motorsRight.attach(13);
    motorsRight.writeMicroseconds(1500);

    // Steering servos
    frontLeft.attach(14);
    frontRight.attach(15);
    backLeft.attach(16);
    backRight.attach(17);

    frontLeft.writeMicroseconds(1500);
    frontRight.writeMicroseconds(1500);
    backLeft.writeMicroseconds(1500);
    backRight.writeMicroseconds(1500);

    // 6 Axis Arm Servos
    arm_servos[0].attach(18);
    arm_servos[1].attach(19);
    arm_servos[2].attach(21);
    arm_servos[3].attach(22);
    arm_servos[4].attach(23);
    arm_servos[5].attach(2);

    // Arm start position
    arm_servos[0].writeMicroseconds(1500);
    arm_servos[1].writeMicroseconds(1500);
    arm_servos[2].writeMicroseconds(1000);
    arm_servos[3].writeMicroseconds(1300);
    arm_servos[4].writeMicroseconds(1500);
    arm_servos[5].writeMicroseconds(1500);

    arm_init(arm_servos, ARM_NUM_AXIS);

    ESP_LOGI(TAG, "Ready, core: %d", xPortGetCoreID());
}


void handleRobotArmServo(ArmAxis arm_axis, uint16_t channel) {
  
  uint16_t speed;
  uint16_t signal = filter_signal(rc_values[channel]);
  if (signal < 1500 && signal > 0) {
    signal = 2 * 1500 - signal;
    speed = map(signal, 1500, 2000, ARM_MIN_SPEED, ARM_MAX_SPEED);
    arm_move_axis_us(arm_axis, DEFAULT_uS_LOW, speed);
  } else if (signal > 1500) {
    speed = map(signal, 1500, 2000, ARM_MIN_SPEED, ARM_MAX_SPEED);
    arm_move_axis_us(arm_axis, DEFAULT_uS_HIGH, speed);
  } else {
    arm_pause(arm_axis);
  }
}

void setMotorInReverseMode(Servo motor) {
  motor.writeMicroseconds(1400);
  delay(100);
  motor.writeMicroseconds(1500);
  delay(100);
}

void handleMoveMotors(uint16_t signal) {
  //LOGLN(signal);
  switch (currentRoverMode) {
    case DRIVE_TURN_NORMAL:
        if (signal < 1500 && signal > 0) { // MOTOR_Backward        
          // When going from MOTOR_forward/MOTOR_idle to reverse, motors (ESC) need to be set in reverse mode
          if (motorState == MOTOR_IDLE) {
              setMotorInReverseMode(motorsLeft);
              setMotorInReverseMode(motorsRight);
              motorState = MOTOR_BACKWARD;
          }
        
      } else if (signal > 1500) {
        motorState = MOTOR_FORWARD;
      } else {
        motorState = MOTOR_IDLE;
      }
      motorsLeft.writeMicroseconds(signal);
      motorsRight.writeMicroseconds(signal);
      break;
    case DRIVE_TURN_SPIN:
      {
        uint16_t diff = abs((int16_t)1500 - (int16_t)signal);

        if (signal < 1500 && signal > 0) { // Spin Anticlockwise
          if (motorState == MOTOR_IDLE) {
            setMotorInReverseMode(motorsLeft);
            motorState = MOTOR_FORWARD;
          }
          motorsRight.writeMicroseconds(1500 + diff);
          motorsLeft.writeMicroseconds(1500 - diff);
        } else if (signal > 1500) {
          if (motorState == MOTOR_IDLE) {
            setMotorInReverseMode(motorsRight);
            motorState = MOTOR_FORWARD;
          }
          motorsRight.writeMicroseconds(1500 - diff); // Spin Clockwise
          motorsLeft.writeMicroseconds(1500 + diff);
        } else {
          motorState = MOTOR_IDLE;
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
      frontLeft.writeMicroseconds(1500 - diff);
      frontRight.writeMicroseconds(1500 - diff);
      backLeft.writeMicroseconds(1500 + diff);
      backRight.writeMicroseconds(1500 + diff);
  } else if (signal > 1500) { // Right turn
      frontLeft.writeMicroseconds(1500 + diff);
      frontRight.writeMicroseconds(1500 + diff);
      backLeft.writeMicroseconds(1500 - diff);
      backRight.writeMicroseconds(1500 - diff);
  } else {
    frontLeft.writeMicroseconds(1500);
    frontRight.writeMicroseconds(1500);
    backLeft.writeMicroseconds(1500);
    backRight.writeMicroseconds(1500);
  }
}

void steerSpin(uint16_t signal) {
  frontLeft.writeMicroseconds(2000);
  frontRight.writeMicroseconds(1000);
  backLeft.writeMicroseconds(1000);
  backRight.writeMicroseconds(2000);
}

void handleSteer(void) {
  uint16_t signal = filter_signal(rc_values[RC_STEER_CHANNEL]);
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
  
  return;
  switch (currentRoverMode) {
    case DRIVE_TURN_NORMAL:
    case DRIVE_TURN_SPIN:
    {
      uint16_t signal = 1500;

      rc_values[RC_STEER_CHANNEL][rcValueIndex] = rc_receiver_rmt_get_val(RC_STEER_CHANNEL);
      rc_values[RC_MOTOR_CHANNEL][rcValueIndex] = rc_receiver_rmt_get_val(RC_MOTOR_CHANNEL);
      handleIfControllerDisconnected(rc_values[RC_STEER_CHANNEL][rcValueIndex]);

      if (currentRoverMode == DRIVE_TURN_NORMAL) {
        signal = filter_signal(rc_values[RC_MOTOR_CHANNEL]);
      } else if (currentRoverMode == DRIVE_TURN_SPIN) {
        signal = filter_signal(rc_values[RC_STEER_CHANNEL]);
      }
      handleMoveMotors(signal);
      handleSteer();
      break;
    }
    case ROBOT_ARM:
      if (currentArmMode == ARM_MODE_MOVE) {
        rc_values[RC_SERVO1_CHANNEL][rcValueIndex] = rc_receiver_rmt_get_val(RC_SERVO1_CHANNEL);
        rc_values[RC_SERVO2_CHANNEL][rcValueIndex] = rc_receiver_rmt_get_val(RC_SERVO2_CHANNEL);
        rc_values[RC_SERVO3_CHANNEL][rcValueIndex] = rc_receiver_rmt_get_val(RC_SERVO3_CHANNEL);
        rc_values[RC_SERVO4_CHANNEL][rcValueIndex] = rc_receiver_rmt_get_val(RC_SERVO4_CHANNEL);
        handleIfControllerDisconnected(rc_values[RC_SERVO1_CHANNEL][rcValueIndex]);

        handleRobotArmServo(ARM_AXIS_1, RC_SERVO1_CHANNEL);
        handleRobotArmServo(ARM_AXIS_2, RC_SERVO2_CHANNEL);
        handleRobotArmServo(ARM_AXIS_3, RC_SERVO3_CHANNEL);
        handleRobotArmServo(ARM_AXIS_4, RC_SERVO4_CHANNEL);
      } else if (currentArmMode == ARM_MODE_GRIPPER) {
        rc_values[RC_SERVO5_CHANNEL][rcValueIndex] = rc_receiver_rmt_get_val(RC_SERVO5_CHANNEL);
        rc_values[RC_SERVO6_CHANNEL][rcValueIndex] = rc_receiver_rmt_get_val(RC_SERVO6_CHANNEL);
        handleIfControllerDisconnected(rc_values[RC_SERVO5_CHANNEL][rcValueIndex]);

        handleRobotArmServo(ARM_AXIS_5, RC_SERVO5_CHANNEL);
        handleRobotArmServo(ARM_AXIS_6, RC_SERVO6_CHANNEL);
      }
      break;
  }
  rcValueIndex = (rcValueIndex + 1) % RC_FILTER_SAMPLES;
  vTaskDelay(20 / portTICK_PERIOD_MS);
}