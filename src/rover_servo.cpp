#include <Arduino.h>
#include "rover_servo.h"
#include "freertos/FreeRTOS.h"
#include "freertos/list.h"
#include "ESP32Servo.h"
#include "rover_config.h"
#include "esp_timer.h"
#include "esp_err.h"

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define ADAFRUIT_PWM_EXPANDER_ADDR  0x40

#define SERVO_UPDATE_INTERVAL_MS  10

typedef enum ServoDirection {
  DIRECTION_POSITIVE,
  DIRECTION_NEGATIVE
} ServoDirection;

typedef struct ServoState {
  uint16_t      min;
  uint16_t      max;
  uint16_t      current_pos;
  uint16_t      start_pos;
  uint16_t      end_pos;
  uint8_t       speed;
  uint32_t      num_moves;
  uint32_t      steps_per_move;
  uint32_t      num_moves_finished;
  ServoDirection direction;
  bool          isPaused;
  xList         path;
} ServoState;

static void update_servo_positions_timer_cb(void* arg);
static void reset_moving_servo(RoverServoId servoId);
static uint16_t calibrated_servo_map(RoverServoId servoId, uint16_t us, uint16_t in_low, uint16_t in_high, uint16_t out_low, uint16_t out_high);

static xSemaphoreHandle position_update_sem_handle;
static xSemaphoreHandle i2cSemaphoreHandle;

static Adafruit_PWMServoDriver servo_driver = Adafruit_PWMServoDriver(ADAFRUIT_PWM_EXPANDER_ADDR, Wire);
static bool isInitialized = false;
static ServoState servo_states[SERVO_LAST];
static esp_timer_handle_t servo_update_timer;

void rover_servo_init(xSemaphoreHandle i2cSemHandle) {
  if (!isInitialized) {
    isInitialized = true;
    memset(&servo_states, 0, sizeof(servo_states));
    for (uint8_t i = 0; i < SERVO_LAST; i++) {
      servo_states[i].current_pos = RC_CENTER;
    }
    i2cSemaphoreHandle = i2cSemHandle;
    assert(pdTRUE == xSemaphoreTake(i2cSemaphoreHandle, pdMS_TO_TICKS(100)));
    servo_driver.begin();
    servo_driver.setPWMFreq(60);
    xSemaphoreGive(i2cSemaphoreHandle);
    position_update_sem_handle = xSemaphoreCreateBinary();
    assert(position_update_sem_handle);
    xSemaphoreGive(position_update_sem_handle);

    esp_timer_create_args_t periodic_timer_args;
    periodic_timer_args.callback = &update_servo_positions_timer_cb;
    periodic_timer_args.name = "update_servo_positions";

    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &servo_update_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(servo_update_timer, SERVO_UPDATE_INTERVAL_MS * 1000));
  }
}

void rover_servo_write(RoverServoId servoId, uint16_t us, bool full_range)
{
  assert(servoId < SERVO_LAST);
  assert(us >= RC_LOW);
  assert(us <= RC_HIGH);
  xSemaphoreTake(position_update_sem_handle, portMAX_DELAY);
  uint16_t pwm_value;
  if (full_range) {
    pwm_value = map(us, RC_LOW, RC_HIGH, SERVOMIN_FULL_RANGE, SERVOMAX_FULL_RANGE);
  } else {
    pwm_value = calibrated_servo_map(servoId, us, RC_LOW, RC_HIGH, SERVOMIN, SERVOMAX);
  }

  reset_moving_servo(servoId);
  servo_states[servoId].current_pos = us;

  assert(pdTRUE == xSemaphoreTake(i2cSemaphoreHandle, pdMS_TO_TICKS(1000)));
  servo_driver.setPWM((uint8_t)servoId, 0, pwm_value);
  xSemaphoreGive(i2cSemaphoreHandle);
  xSemaphoreGive(position_update_sem_handle);
}

void rover_servo_move(RoverServoId servoId, uint16_t pos, uint8_t speed)
{
  assert(servoId < SERVO_LAST);
  ServoState* servo_state = &servo_states[servoId];
  uint32_t steps_to_move;
  uint32_t internal_speed;
  assert(pos >= RC_LOW);
  assert(pos <= RC_HIGH);
  assert(speed >= SERVO_MIN_SPEED);
  assert(speed <= SERVO_MAX_SPEED);
  internal_speed = SERVO_MAX_SPEED + 1 - speed; // Speed is inverted internally 1 fastest, 10 slowest

  if (pos == servo_state->current_pos || (pos == servo_state->end_pos && internal_speed == servo_state->speed)) return;
  xSemaphoreTake(position_update_sem_handle, portMAX_DELAY);
  servo_state->isPaused = false;
  servo_state->end_pos = pos;
  servo_state->start_pos = servo_state->current_pos;
  servo_state->speed = internal_speed;
  steps_to_move = abs(servo_state->end_pos - servo_state->current_pos);
  servo_state->num_moves = (steps_to_move / SERVO_UPDATE_INTERVAL_MS);
  if (servo_state->num_moves == 0) {
    servo_state->num_moves = 1;
  }
  servo_state->steps_per_move = steps_to_move / servo_state->num_moves;
  servo_state->num_moves_finished = 0;
  servo_state->direction = (servo_state->end_pos > servo_state->current_pos) ? DIRECTION_POSITIVE : DIRECTION_NEGATIVE;
  xSemaphoreGive(position_update_sem_handle);
}

void rover_servo_pause(RoverServoId servoId)
{
  assert(servoId < SERVO_LAST);
  servo_states[servoId].isPaused = true;
}

void rover_servo_resume(RoverServoId servoId)
{
  assert(servoId < SERVO_LAST);
  servo_states[servoId].isPaused = false;
}

static void update_servo_positions_timer_cb(void* arg)
{
  ServoState* axis;
  uint32_t next_servo_pos;
  for (uint8_t i = 0; i < SERVO_LAST; i++) {
    axis = &servo_states[i];
    if (axis->num_moves == 0) continue;
    if (axis->isPaused) continue;
    
    xSemaphoreTake(position_update_sem_handle, portMAX_DELAY);
    axis->num_moves_finished++;
    if (axis->num_moves_finished % axis->speed == 0) {
      // Check if last move
      if (abs(axis->current_pos - axis->end_pos) <= axis->steps_per_move) {
        axis->current_pos = axis->end_pos;
        next_servo_pos = axis->end_pos;
        axis->num_moves = 0;
        axis->num_moves_finished = 0;
        axis->start_pos = axis->current_pos;
      } else if (axis->direction == DIRECTION_POSITIVE) {
        next_servo_pos = axis->current_pos + axis->steps_per_move;
        axis->current_pos = axis->current_pos + axis->steps_per_move;
      } else {
        next_servo_pos = axis->current_pos - axis->steps_per_move;
        axis->current_pos = axis->current_pos - axis->steps_per_move;
      }
      xSemaphoreGive(position_update_sem_handle);
      rover_servo_write((RoverServoId)i, next_servo_pos, true);
    } else {
      xSemaphoreGive(position_update_sem_handle);
    }
  }
}

static void reset_moving_servo(RoverServoId servoId)
{
  assert(servoId < SERVO_LAST);
  memset(&servo_states[servoId], 0 , sizeof(ServoState));
  servo_states[servoId].isPaused = true;
}

static uint16_t calibrated_servo_map(RoverServoId servoId, uint16_t us, uint16_t in_low, uint16_t in_high, uint16_t out_low, uint16_t out_high)
{
  uint16_t val = map(us, in_low, in_high, out_low, out_high);
  
  switch (servoId) {
    case SERVO_FRONT_LEFT:
      val += SERVO_FRONT_LEFT_OFFSET;
      break;
    case SERVO_FRONT_RIGHT:
      val += SERVO_FRONT_RIGHT_OFFSET;
      break;
    case SERVO_BACK_LEFT:
      val += SERVO_BACK_LEFT_OFFSET;
      break;
    case SERVO_BACK_RIGHT:
      val += SERVO_BACK_RIGHT_OFFSET;
      break;
    default:
      break;
  }

  if (val < SERVOMIN_FULL_RANGE) {
    val = SERVOMIN_FULL_RANGE;
  } else if (val > SERVOMAX_FULL_RANGE) {
    val = SERVOMAX_FULL_RANGE;
  }

  return val;
}