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
#include <ESP32Servo.h>
#include "Wire.h"
#include "rc_receiver_rmt.h"
#include "wifi_controller.h"
#include "switch_checker.h"
#include "arm.h"
#include "gyro_accel_sensor.h"
#include "rover_servo.h"

#define DEBUG

#ifdef DEBUG
#define LOG Serial.print
#define LOGF Serial.printf
#define LOGLN Serial.println
#else
#define LOG(msg)
#define LOGF(...)
#define LOGLN(msg)
#endif

static void handle_wifi_controller_status(WifiControllerStatus status);

enum MotorDirection {
  MOTOR_IDLE,
  MOTOR_FORWARD,
  MOTOR_BACKWARD
};

static uint16_t rc_values[RC_NUM_CHANNELS][RC_FILTER_SAMPLES];
static uint8_t rc_value_index = 0;

static MotorDirection motor_state = MOTOR_IDLE;
static RoverMode current_rover_mode = DRIVE_TURN_NORMAL;
static ArmMode current_arm_mode = ARM_MODE_MOVE;

static bool wifi_control_enabled = false;

static Servo motors_left;
static Servo motors_right;

static Servo front_left;
static Servo front_right;
static Servo back_left;
static Servo back_right;

static WifiControllerStatus current_wifi_status = WIFI_CONTROLLER_ERROR;


static uint16_t filter_signal(uint16_t* signals) {
  uint32_t signal = 0;
  for (uint8_t i = 0; i < RC_FILTER_SAMPLES; i++) {
    signal += signals[i];
  }
  signal = signal / RC_FILTER_SAMPLES;
  
  if (signal <= 800) return RC_CENTER; // If we are unlucky in timing when controller disconnect we might get some random low signal
  if (signal > 800 && signal < 1010) return RC_LOW;
  if (signal > 1990) return RC_HIGH;
  if (signal > 1480 && signal < 1520) return RC_CENTER;
  return signal;;
}

static void handle_rover_mode_changed(RoverMode mode) {
  LOGF("ROVER MODE CHANGED: %d\n", mode);
  current_rover_mode = mode;
}

static void handle_arm_mode_changed(ArmMode mode) {
  LOGF("ARM MODE CHANGED: %d\n", mode);
  current_arm_mode = mode;
}

static void handle_controller_disconnected(uint16_t last_sampled_signal) {
  // Set all RC values to mid position if signal is 0 (controller disconnected)
  if (last_sampled_signal == 0) {
    for (uint8_t channel = 0; channel < RC_NUM_CHANNELS; channel++) {
      for (uint8_t sample = 0; sample < RC_FILTER_SAMPLES; sample++) {
        rc_values[channel][sample] = RC_CENTER;
      }
    }
  }
}

void setup() {
    Serial.begin(SERIAL_PORT_SPEED);
    Wire.begin(ROVER_SDA_PIN, ROVER_SCL_PIN);
    Wire.setClock(400000);

    //gyro_accel_init(true);
    rover_servo_init();
    handle_controller_disconnected(0);

    rc_receiver_rmt_init();
    
    wifi_controller_init("rover", NULL);
    init_switch_checker(RC_LOW, RC_ROVER_MODE_ROVER_CHANNEL, RC_ARM_MODE_ROVER_CHANNEL, &handle_rover_mode_changed, &handle_arm_mode_changed);
    register_connection_callback(&handle_wifi_controller_status);

    motors_left.attach(ROVER_MOTORS_LEFT_PIN);
    motors_left.writeMicroseconds(RC_CENTER);
    motors_right.attach(ROVER_MOTORS_RIGHT_PIN);
    motors_right.writeMicroseconds(RC_CENTER);

    // Steering servos
    rover_servo_write(SERVO_FRONT_LEFT, RC_CENTER);
    rover_servo_write(SERVO_FRONT_RIGHT, RC_CENTER);
    rover_servo_write(SERVO_BACK_LEFT, RC_CENTER);
    rover_servo_write(SERVO_BACK_RIGHT, RC_CENTER);

    arm_init();

    LOGF("Rover Ready! Core: %d", xPortGetCoreID());
}

static void handle_robot_arm_servo(ArmAxis arm_axis, uint16_t channel) {
  
  uint16_t speed;
  uint16_t signal = filter_signal(rc_values[channel]);
  if (signal < RC_CENTER && signal > 0) {
    signal = 2 * RC_CENTER - signal;
    speed = map(signal, RC_CENTER, RC_HIGH, ARM_MIN_SPEED, ARM_MAX_SPEED);
    arm_move_axis_us(arm_axis, RC_LOW, speed);
  } else if (signal > RC_CENTER) {
    speed = map(signal, RC_CENTER, RC_HIGH, ARM_MIN_SPEED, ARM_MAX_SPEED);
    arm_move_axis_us(arm_axis, RC_HIGH, speed);
  } else {
    arm_pause(arm_axis);
  }
}

static void setMotorInReverseMode(Servo motor) {
  motor.writeMicroseconds(1400);
  delay(100);
  motor.writeMicroseconds(RC_CENTER);
  delay(100);
}

static void handleMoveMotors(uint16_t signal) {
  //LOGLN(signal);
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

static void steer_normal(uint16_t signal) {
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

static void steer_spin(uint16_t signal) {
  rover_servo_write(SERVO_FRONT_LEFT, RC_HIGH);
  rover_servo_write(SERVO_FRONT_RIGHT, RC_LOW);
  rover_servo_write(SERVO_BACK_LEFT, RC_HIGH);
  rover_servo_write(SERVO_BACK_RIGHT, RC_LOW);
}

static void handle_steer(void) {
  uint16_t signal = filter_signal(rc_values[RC_STEER_CHANNEL]);
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

static void handle_wifi_controller_status(WifiControllerStatus status)
{
  LOGF("Wifi Controller status: %d\n", status);
  current_wifi_status = status;
  if (status != WIFI_CONTROLLER_CONNECTED) {
    handle_controller_disconnected(0);
    wifi_control_enabled = false;
  } else {
    wifi_control_enabled = true;
  }
}

static uint16_t get_controller_channel_value(uint8_t channel)
{
  uint16_t channel_value = 0;

  if (wifi_control_enabled) {
    channel_value = wifi_controller_get_val(channel);
  } else {
    channel_value = rc_receiver_rmt_get_val(channel);
  }

  if (channel_value == 0) {
    channel_value = RC_CENTER;
  }

  return channel_value;
}

void loop() {
  switch (current_rover_mode) {
    case DRIVE_TURN_NORMAL:
    case DRIVE_TURN_SPIN:
    {
      uint16_t signal = RC_CENTER;

      rc_values[RC_STEER_CHANNEL][rc_value_index] = get_controller_channel_value(RC_STEER_CHANNEL);
      rc_values[RC_MOTOR_CHANNEL][rc_value_index] = get_controller_channel_value(RC_MOTOR_CHANNEL);

      if (current_rover_mode == DRIVE_TURN_NORMAL) {
        signal = filter_signal(rc_values[RC_MOTOR_CHANNEL]);
      } else if (current_rover_mode == DRIVE_TURN_SPIN) {
        signal = filter_signal(rc_values[RC_STEER_CHANNEL]);
      }
      handleMoveMotors(signal);
      handle_steer();
      break;
    }
    case ROBOT_ARM:
      if (current_arm_mode == ARM_MODE_MOVE) {
        rc_values[RC_SERVO1_CHANNEL][rc_value_index] = get_controller_channel_value(RC_SERVO1_CHANNEL);
        rc_values[RC_SERVO2_CHANNEL][rc_value_index] = get_controller_channel_value(RC_SERVO2_CHANNEL);
        rc_values[RC_SERVO3_CHANNEL][rc_value_index] = get_controller_channel_value(RC_SERVO3_CHANNEL);
        rc_values[RC_SERVO4_CHANNEL][rc_value_index] = get_controller_channel_value(RC_SERVO4_CHANNEL);

        handle_robot_arm_servo(ARM_AXIS_1, RC_SERVO1_CHANNEL);
        handle_robot_arm_servo(ARM_AXIS_2, RC_SERVO2_CHANNEL);
        handle_robot_arm_servo(ARM_AXIS_3, RC_SERVO3_CHANNEL);
        handle_robot_arm_servo(ARM_AXIS_4, RC_SERVO4_CHANNEL);
      } else if (current_arm_mode == ARM_MODE_GRIPPER) {
        rc_values[RC_SERVO5_CHANNEL][rc_value_index] = get_controller_channel_value(RC_SERVO5_CHANNEL);
        rc_values[RC_SERVO6_CHANNEL][rc_value_index] = get_controller_channel_value(RC_SERVO6_CHANNEL);

        handle_robot_arm_servo(ARM_AXIS_5, RC_SERVO5_CHANNEL);
        handle_robot_arm_servo(ARM_AXIS_6, RC_SERVO6_CHANNEL);
      }
      break;
  }
  rc_value_index = (rc_value_index + 1) % RC_FILTER_SAMPLES;
  vTaskDelay(pdMS_TO_TICKS(20));
}