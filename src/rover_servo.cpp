#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/list.h"
#include "ESP32Servo.h"
#include "arm.h"
#include "rover_config.h"
#include "esp_timer.h"
#include "esp_err.h"

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define ADAFRUIT_PWM_EXPANDER_ADDR  0x40


static Adafruit_PWMServoDriver servo_driver = Adafruit_PWMServoDriver(ADAFRUIT_PWM_EXPANDER_ADDR, Wire);
static bool isInitialized = false;

void rover_servo_init() {
  if (!isInitialized) {
    isInitialized = true;
    servo_driver.begin();
    servo_driver.setPWMFreq(60);
  }
}

void rover_servo_write(RoverServo axis, uint16_t us)
{
  // TODO need Semaphore here as different tasks can use this function
  assert(us >= RC_LOW);
  assert(us <= RC_HIGH);
  uint16_t pwm_value = map(us, RC_LOW, RC_HIGH, SERVOMIN, SERVOMAX);
  servo_driver.setPWM((uint8_t)axis, 0, pwm_value);
}