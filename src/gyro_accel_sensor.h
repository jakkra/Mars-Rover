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

void gyro_accel_init(xSemaphoreHandle i2cSemHandle, bool calibrate);
GyroAccelData gyro_accel_get_current();