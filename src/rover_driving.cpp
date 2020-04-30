#include "esp_system.h"
#include "rover_driving.h"
#include "rover_config.h"
#include "rover_servo.h"
#include <ESP32Servo.h>

static void steer_normal(uint16_t signal);
static void steer_spin(uint16_t signal);

enum MotorDirection {
  MOTOR_IDLE,
  MOTOR_FORWARD,
  MOTOR_BACKWARD
};

static Servo motors_left;
static Servo motors_right;

static MotorDirection motor_state = MOTOR_IDLE;
static RoverMode current_rover_mode = DRIVE_TURN_NORMAL;

void rover_driving_init(void)
{
    motors_left.attach(ROVER_MOTORS_LEFT_PIN);
    motors_left.writeMicroseconds(RC_CENTER);
    motors_right.attach(ROVER_MOTORS_RIGHT_PIN);
    motors_right.writeMicroseconds(RC_CENTER);

    // Steering servos
    rover_servo_write(SERVO_FRONT_LEFT, RC_CENTER);
    rover_servo_write(SERVO_FRONT_RIGHT, RC_CENTER);
    rover_servo_write(SERVO_BACK_LEFT, RC_CENTER);
    rover_servo_write(SERVO_BACK_RIGHT, RC_CENTER);
}

static void setMotorInReverseMode(Servo motor) {
  motor.writeMicroseconds(1400);
  delay(100);
  motor.writeMicroseconds(RC_CENTER);
  delay(100);
}

void rover_driving_set_drive_mode(RoverMode mode)
{
    current_rover_mode = mode;
}

void rover_driving_move(uint16_t signal) {
  switch (current_rover_mode) {
    case DRIVE_TURN_NORMAL:
        if (signal < RC_CENTER && signal > 0) { // MOTOR_Backward        
          // When going from MOTOR_forward/MOTOR_idle to reverse, motors (ESC) need to be set in reverse mode
          if (motor_state == MOTOR_IDLE) {
              setMotorInReverseMode(motors_left);
              setMotorInReverseMode(motors_right);
              motor_state = MOTOR_BACKWARD;
          }
        
      } else if (signal > RC_CENTER) {
        motor_state = MOTOR_FORWARD;
      } else {
        motor_state = MOTOR_IDLE;
      }
      motors_left.writeMicroseconds(signal);
      motors_right.writeMicroseconds(signal);
      break;
    case DRIVE_TURN_SPIN:
      {
        uint16_t diff = abs((int16_t)RC_CENTER - (int16_t)signal);

        if (signal < RC_CENTER && signal > 0) { // Spin Anticlockwise
          if (motor_state == MOTOR_IDLE) {
            setMotorInReverseMode(motors_left);
            motor_state = MOTOR_FORWARD;
          }
          motors_right.writeMicroseconds(RC_CENTER + diff);
          motors_left.writeMicroseconds(RC_CENTER - diff);
        } else if (signal > RC_CENTER) {
          if (motor_state == MOTOR_IDLE) {
            setMotorInReverseMode(motors_right);
            motor_state = MOTOR_FORWARD;
          }
          motors_right.writeMicroseconds(RC_CENTER - diff); // Spin Clockwise
          motors_left.writeMicroseconds(RC_CENTER + diff);
        } else {
          motor_state = MOTOR_IDLE;
          motors_left.writeMicroseconds(signal);
          motors_right.writeMicroseconds(signal);
        }
        break;
      }
    default:
      break;
  }

}

void rover_driving_steer(uint16_t signal) {
  switch (current_rover_mode) {
    case DRIVE_TURN_NORMAL:
      steer_normal(signal);
      break;
    case DRIVE_TURN_SPIN:
      steer_spin(signal);
      break;
    default:
      break;
  }
}

static void steer_normal(uint16_t signal)
{
  uint16_t diff = abs((int16_t)RC_CENTER - (int16_t)signal);
  if (signal < RC_CENTER && signal > 0) { // Left turn
      rover_servo_write(SERVO_FRONT_LEFT, RC_CENTER - diff);
      rover_servo_write(SERVO_FRONT_RIGHT, RC_CENTER - diff);
      rover_servo_write(SERVO_BACK_LEFT, RC_CENTER + diff);
      rover_servo_write(SERVO_BACK_RIGHT, RC_CENTER + diff);
  } else if (signal > RC_CENTER) { // Right turn
      rover_servo_write(SERVO_FRONT_LEFT, RC_CENTER + diff);
      rover_servo_write(SERVO_FRONT_RIGHT, RC_CENTER + diff);
      rover_servo_write(SERVO_BACK_LEFT, RC_CENTER - diff);
      rover_servo_write(SERVO_BACK_RIGHT, RC_CENTER - diff);
  } else {
    rover_servo_write(SERVO_FRONT_LEFT, RC_CENTER);
    rover_servo_write(SERVO_FRONT_RIGHT, RC_CENTER);
    rover_servo_write(SERVO_BACK_LEFT, RC_CENTER);
    rover_servo_write(SERVO_BACK_RIGHT, RC_CENTER);
  }
}

static void steer_spin(uint16_t signal)
{
  rover_servo_write(SERVO_FRONT_LEFT, RC_HIGH);
  rover_servo_write(SERVO_FRONT_RIGHT, RC_LOW);
  rover_servo_write(SERVO_BACK_LEFT, RC_HIGH);
  rover_servo_write(SERVO_BACK_RIGHT, RC_LOW);
}
