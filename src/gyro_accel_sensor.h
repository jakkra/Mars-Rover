#pragma once

typedef struct GyroAccelYpr
{
  float yaw;
  float pitch;
  float roll;
} GyroAccelYpr;


void gyro_accel_init(bool calibrate);
GyroAccelYpr gyro_accel_get_current();