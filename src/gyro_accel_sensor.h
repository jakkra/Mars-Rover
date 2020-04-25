#pragma once

typedef struct GyroAccelData
{
  float temp;
  float accX;
  float accY;
  float accZ;
  float gyroX;
  float gyroY;
  float gyroZ;
  float gyroAngleX;
  float gyroAngleY;
  float gyroAngleZ;
  float angleX;
  float angleY;
  float angleZ;
} GyroAccelData;

typedef void(OnData(GyroAccelData* data));


void gyro_accel_init(xSemaphoreHandle i2c_sem_handle, bool calibrate, OnData callback, uint32_t period_ms);
GyroAccelData gyro_accel_get_current();