#pragma once

#define SERIAL_PORT_SPEED 115200

#define RC_NUM_CHANNELS   6
#define RC_FILTER_SAMPLES 2

#define RC_LOW    1000
#define RC_CENTER 1500
#define RC_HIGH   2000

#define RC_CH1  0
#define RC_CH2  1
#define RC_CH3  2
#define RC_CH4  3
#define RC_CH5  4
#define RC_CH6  5

// Input pins connected to each channel on RC Receiver
#define RC_CH1_INPUT  34
#define RC_CH2_INPUT  35
#define RC_CH3_INPUT  4
#define RC_CH4_INPUT  39
#define RC_CH5_INPUT  25
#define RC_CH6_INPUT  36

#define RC_STEER_CHANNEL      RC_CH1
#define RC_MOTOR_CHANNEL      RC_CH2
#define RC_HEAD_YAW_CHANNEL   RC_CH3
#define RC_HEAD_PITCH_CHANNEL RC_CH4

#define RC_SERVO1_CHANNEL RC_CH1
#define RC_SERVO2_CHANNEL RC_CH2
#define RC_SERVO3_CHANNEL RC_CH4
#define RC_SERVO4_CHANNEL RC_CH3
#define RC_SERVO5_CHANNEL RC_CH1
#define RC_SERVO6_CHANNEL RC_CH2

#define RC_ROVER_MODE_ROVER_CHANNEL RC_CH5
#define RC_ARM_MODE_ROVER_CHANNEL   RC_CH6


#define ARM_NUM_AXIS 6

#ifdef NINA_W10
#define ROVER_SCL_PIN               5
#define ROVER_SDA_PIN               23
#else
#define ROVER_SCL_PIN               22
#define ROVER_SDA_PIN               21
#endif

#define SERVOMIN  250 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  500 // this is the 'maximum' pulse length count (out of 4096)

#define SERVOMIN_FULL_RANGE  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX_FULL_RANGE  600 // this is the 'maximum' pulse length count (out of 4096)

#define ROVER_MOTORS_LEFT_PIN       17
#define ROVER_MOTORS_RIGHT_PIN      13

#define ROVER_SETTINGS_SWITCH_1     2
#define ROVER_SETTINGS_SWITCH_2     23

// Offset used in rover_servo to make 
#define SERVO_FRONT_LEFT_OFFSET     (+20)
#define SERVO_FRONT_RIGHT_OFFSET    (-15)
#define SERVO_BACK_LEFT_OFFSET      (+5)
#define SERVO_BACK_RIGHT_OFFSET     (-25)

const uint16_t config_default_ch_values[RC_NUM_CHANNELS] = { RC_CENTER, RC_CENTER, RC_CENTER, RC_CENTER, RC_LOW, RC_CENTER };