#pragma once

#define SERIAL_PORT_SPEED 115200

#define RC_NUM_CHANNELS   6
#define RC_FILTER_SAMPLES 2

#define RECEIVER_CH_CENTER 1500

#define RC_CH1  0
#define RC_CH2  1
#define RC_CH3  2
#define RC_CH4  3
#define RC_CH5  4
#define RC_CH6  5

// Input pins connected to each channel on RC Receiver
#define RC_CH1_INPUT  34
#define RC_CH2_INPUT  35
#define RC_CH3_INPUT  32
#define RC_CH4_INPUT  33
#define RC_CH5_INPUT  25
#define RC_CH6_INPUT  26

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

#define RC_ARM_MODE_ROVER_CHANNEL   RC_CH5
#define RC_ROVER_MODE_ROVER_CHANNEL RC_CH6


#define ARM_NUM_AXIS 6

#define ROVER_SCL_PIN               27
#define ROVER_SDA_PIN               22
#define ROVER_GYRO_ACCEL_INT_PIN    34

#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)