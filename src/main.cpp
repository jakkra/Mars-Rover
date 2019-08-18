#include <Arduino.h>
#include "ServoEasing.h"

typedef struct {
  uint8_t forward_value;
  uint8_t endstop_min;
  uint8_t endstop_max;
  Servo servo;
} calibrated_servo;

#define SERIAL_PORT_SPEED 9600
#define RC_NUM_CHANNELS  6

#define SERVO_MOVE_SPEED 60
#define MIN_SERVO_ANGLE 20
#define MAX_SERVO_ANGLE 160

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

uint16_t rc_values[RC_NUM_CHANNELS];


ServoEasing Servo1;
ServoEasing Servo2;
ServoEasing Servo3;

uint16_t filterSignal(uint16_t signal) {
  if (signal < 1000) return 1000;
  if (signal > 2000) return 2000;
  if (signal > 1460 && signal < 1540) return 1500;
  return signal;
}

void setup() {
  Serial.begin(SERIAL_PORT_SPEED);

  pinMode(RC_CH1_INPUT, INPUT);
  pinMode(RC_CH2_INPUT, INPUT);
  pinMode(RC_CH3_INPUT, INPUT);
  pinMode(RC_CH4_INPUT, INPUT);
  pinMode(RC_CH5_INPUT, INPUT);
  pinMode(RC_CH6_INPUT, INPUT);


  Servo1.attach(8);
  Servo2.attach(9);
  Servo3.attach(10);
  Servo1.setSpeed(30);
  Servo2.setSpeed(30);
  Servo3.setSpeed(30);
  Servo1.setEaseTo(90);
  Servo2.setEaseTo(90);
  Servo3.startEaseTo(90);
  while (Servo1.isMoving() || Servo2.isMoving() || Servo3.isMoving());
  Serial.println("Ready");
}

void handleRcSignal(ServoEasing* servo, uint16_t channel) {
  uint16_t speed;
  uint16_t signal = filterSignal(rc_values[channel]);
  if (signal < 1500) {
    Serial.print("CH:"); Serial.print(channel); Serial.print(" "); Serial.print(rc_values[channel]); Serial.print("\t");
    speed = map(signal, 1000, 1500, 0, MAX_SERVO_ANGLE);
    speed = max(MAX_SERVO_ANGLE - speed, 1);
    Serial.print("Signal: ");
    Serial.print(signal);
    Serial.print("Speed: ");
    Serial.println(speed);
    servo->startEaseTo(MIN_SERVO_ANGLE, speed);
  } else if (signal > 1500) {
    Serial.print("CH:"); Serial.print(channel); Serial.print(" "); Serial.print(rc_values[channel]); Serial.print("\t");
    speed = map(signal, 1500, 2000, 0, MAX_SERVO_ANGLE);
    speed = max(speed, 1);
    Serial.print("Signal: ");
    Serial.print(signal);
    Serial.print("Speed: ");
    Serial.println(speed);
    servo->startEaseTo(MAX_SERVO_ANGLE, speed);
  } else {
    servo->mServoMoves = false;
    if (!isOneServoMoving()) {
      stopAllServos();
    }
  }
}

void loop() {
  
  rc_values[RC_CH1] = pulseIn(RC_CH1_INPUT, HIGH);
  rc_values[RC_CH2] = pulseIn(RC_CH2_INPUT, HIGH);
  rc_values[RC_CH3] = pulseIn(RC_CH3_INPUT, HIGH);
  rc_values[RC_CH4] = pulseIn(RC_CH4_INPUT, HIGH);
  rc_values[RC_CH5] = pulseIn(RC_CH5_INPUT, HIGH);
  rc_values[RC_CH6] = pulseIn(RC_CH6_INPUT, HIGH);

  handleRcSignal(&Servo1, RC_CH1);
  handleRcSignal(&Servo2, RC_CH2);
  handleRcSignal(&Servo3, RC_CH3);
}