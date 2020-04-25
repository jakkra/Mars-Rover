#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Arduino.h>
#include "esp_log.h"
#include "gyro_accel_sensor.h"
#include "rover_config.h"
#include "wifi_controller.h"

#include <Wire.h>
#include <MPU6050_tockn.h>

//#define DEBUG_ACCEL

static void read_mpu(void* params);

static MPU6050 mpu6050(Wire);
static xSemaphoreHandle update_accel_sem;
static xSemaphoreHandle i2c_sem;
static GyroAccelData current_val;
static OnData* callback;
static uint32_t read_period_ms;

void gyro_accel_init(xSemaphoreHandle i2c_sem_handle, bool calibrate, OnData* cb, uint32_t period_ms)
{
  BaseType_t status_bs;
  TaskHandle_t xHandle = NULL;

  i2c_sem = i2c_sem_handle;
  callback = cb;
  read_period_ms = period_ms;

  status_bs = xSemaphoreTake(i2c_sem, pdMS_TO_TICKS(100));
  assert(status_bs == pdTRUE);
  mpu6050.begin();
  if (calibrate) {
    mpu6050.calcGyroOffsets(true, 0, 0);
  }
  xSemaphoreGive(i2c_sem);

  update_accel_sem = xSemaphoreCreateBinary();
  assert(update_accel_sem != NULL);
  xSemaphoreGive(update_accel_sem);

  status_bs = xTaskCreate(read_mpu, "GyroReader", 2048, NULL, tskIDLE_PRIORITY, &xHandle);
  assert(status_bs == pdPASS);
}

GyroAccelData gyro_accel_get_current() {
  GyroAccelData temp;
  xSemaphoreTake(update_accel_sem, portMAX_DELAY);
  memcpy(&temp, &current_val, sizeof(current_val));
  xSemaphoreGive(update_accel_sem);
  return temp;
}

static void read_mpu(void* params)
{
  while (1) {
    if (pdTRUE == xSemaphoreTake(i2c_sem, pdMS_TO_TICKS(100))) {
      mpu6050.update();
      xSemaphoreGive(i2c_sem);
      xSemaphoreTake(update_accel_sem, portMAX_DELAY);
      current_val.temp = mpu6050.getTemp();
      current_val.accX = mpu6050.getAccX();
      current_val.accY = mpu6050.getAccY();
      current_val.accZ = mpu6050.getAccZ();
      current_val.gyroX = mpu6050.getGyroX();
      current_val.gyroY = mpu6050.getGyroY();
      current_val.gyroZ = mpu6050.getGyroZ();
      current_val.gyroAngleX = mpu6050.getGyroAngleX();
      current_val.gyroAngleY = mpu6050.getGyroAngleY();
      current_val.gyroAngleZ = mpu6050.getGyroAngleZ();
      current_val.angleX = mpu6050.getAngleX();
      current_val.angleY = mpu6050.getAngleY();
      current_val.angleZ = mpu6050.getAngleZ();
      GyroAccelData temp;
      memcpy(&temp, &current_val, sizeof(GyroAccelData));
      xSemaphoreGive(update_accel_sem);
      callback(&temp);
#ifdef DEBUG_ACCEL
      Serial.print("angleX : ");Serial.print(current_val.angleX);
      Serial.print("\tangleY : ");Serial.print(current_val.angleY);
      Serial.print("\tangleZ : ");Serial.println(current_val.angleZ);
#endif
    }
    vTaskDelay(pdMS_TO_TICKS(read_period_ms));
  }
}