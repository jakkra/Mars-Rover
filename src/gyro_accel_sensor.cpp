#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Arduino.h>
#include "esp_log.h"
#include "gyro_accel_sensor.h"
#include "rover_config.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

#define DEBUG_ACCEL

static void handle_mpu_data(void* params);
static void dmp_data_ready();

// MPU control/status vars
static xSemaphoreHandle update_accel_sem;
static uint8_t packetSize = 0;
volatile bool mpu_int_triggered = false;
static gyro_accel_val current_val;

MPU6050 mpu;

void gyro_accel_init(bool calibrate) {
  uint8_t status;
  BaseType_t status_bs;
  TaskHandle_t xHandle = NULL;

  mpu.initialize();
  pinMode(ROVER_GYRO_ACCEL_INT_PIN, INPUT);
  
  if (!mpu.testConnection()) {
    printf("Failed init MPU6050\n");
  }

  status = mpu.dmpInitialize();
  if (status == 0) {
    update_accel_sem = xSemaphoreCreateBinary();
    assert(update_accel_sem != NULL);
    xSemaphoreGive(update_accel_sem);

    // TODO find correct values
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);

    // Calibration
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();

    printf("Start DMP\n");
    mpu.setDMPEnabled(true);

    printf("Enable interrupt on ROVER_GYRO_ACCEL_INT_PIN\n");
    attachInterrupt(digitalPinToInterrupt(ROVER_GYRO_ACCEL_INT_PIN), dmp_data_ready, RISING);

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
    printf("packetsize: %d\n", packetSize);
    status_bs = xTaskCreate(handle_mpu_data, "SwitchChecker", 2048, NULL, tskIDLE_PRIORITY, &xHandle);
    assert(status_bs == pdPASS);

  } else {
    printf("ERROR failed init MPU6050 DMP error: %d\n", status);
  }
}

gyro_accel_ypr gyra_accel_get_current() {
  gyro_accel_ypr temp;
  xSemaphoreTake(update_accel_sem, portMAX_DELAY);
  memcpy(&temp, &current_val, sizeof(current_val));
  xSemaphoreGive(update_accel_sem);
  return temp;
}

static void dmp_data_ready() {
    mpu_int_triggered = true;
}

static void handle_mpu_data(void* params)
{
  uint8_t mpuIntStatus;
  uint16_t fifoCount = 0;
  Quaternion q;
  VectorFloat gravity;
  float ypr[3];
  uint8_t fifoBuffer[64];

  while (1) {
      while (!mpu_int_triggered && fifoCount < packetSize) {
        vTaskDelay(50 * portTICK_PERIOD_MS / 1000);
      }

      mpu_int_triggered = false;
      mpuIntStatus = mpu.getIntStatus();

      fifoCount = mpu.getFIFOCount();

      if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

      } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        mpu.getFIFOBytes(fifoBuffer, packetSize);

        fifoCount -= packetSize;
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
#ifdef DEBUG_ACCEL
        Serial.print("ypr\t");
        Serial.print(ypr[0] * 180/M_PI);
        Serial.print("\t");
        Serial.print(ypr[1] * 180/M_PI);
        Serial.print("\t");
        Serial.println(ypr[2] * 180/M_PI);
#endif
        xSemaphoreTake(update_accel_sem, portMAX_DELAY);
        current_val.yaw = ypr[0];
        current_val.pitch = ypr[1];
        current_val.roll = ypr[2];
        xSemaphoreGive(update_accel_sem);
      }
  }
}