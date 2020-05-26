#include "lora_controller.h"
#include "rover_config.h"
#include "FreeRTOS.h"
#include "esp_log.h"

#include <SPI.h>
#include <LoRa.h>

#define MAX_REGISTRATED_CALLBACKS 2

#define LORA_DATA_NOTIFICATION   1

#define LORA_PACKET_LENGTH        12
#define SCK                       5
#define MISO                      19
#define MOSI                      27
#define SS                        18
#define RST                       14
#define DIO0                      26

static const char* TAG = "LORA_CONTROLLER";

static IRAM_ATTR void lora_availible_isr(void);

static void lora_state_checker(void* args);
static void lora_receive_task(void* args);

static LoraControllerStatusCb* status_callbacks[MAX_REGISTRATED_CALLBACKS];
static uint8_t num_callbacks;
static bool is_initialized = false;
static uint16_t channel_values[RC_NUM_CHANNELS] = {0};
static LoraControllerStatus status = LORA_CONTROLLER_DISCONNECTED;
static uint8_t receive_buf[LORA_PACKET_LENGTH];

static uint32_t last_lora_data_ticks = 0;
static uint32_t last_lora_data_checked_ticks = 0;

static TaskHandle_t task_handle;

void lora_controller_init()
{
  assert(!is_initialized);
  BaseType_t task_create_status;

  memset(channel_values, 0, sizeof(channel_values));
  num_callbacks = 0;

  SPI.begin(SCK, MISO, MOSI, SS);
  LoRa.setPins(SS, RST, DIO0);

  if (!LoRa.begin(868E6)) {
    ESP_LOGE(TAG, "Starting LoRa failed!");
    return;
  }
  LoRa.setFrequency(868E6);
  LoRa.setSpreadingFactor(6);
  LoRa.setSignalBandwidth(250E3);
  LoRa.setCodingRate4(5);
  LoRa.enableCrc();

  LoRa.receive(LORA_PACKET_LENGTH);

  ESP_LOGI(TAG, "Starting LoRa OK!");

  task_create_status = xTaskCreate(lora_state_checker, "lora_state_checker", 2048, NULL, tskIDLE_PRIORITY, NULL);
  assert(task_create_status == pdPASS);

  task_create_status = xTaskCreate(lora_receive_task, "lora_receive", 2048, NULL, 2, &task_handle);
  assert(task_create_status == pdPASS);

  is_initialized = true;

  // Need to take care of LoRa data availible isr as the LoRa library does to much stuff in the ISR
  // causing problems with i2c when ISR happens while i2c runs. Instead we will force it into a context switch out
  // of the ISR before doing SPI stuff.
  pinMode(DIO0, INPUT);
  attachInterrupt(digitalPinToInterrupt(DIO0), lora_availible_isr, RISING);
}

void lora_controller_register_connection_callback(LoraControllerStatusCb* cb)
{
  assert(num_callbacks <= MAX_REGISTRATED_CALLBACKS);
  assert(is_initialized);
  status_callbacks[num_callbacks] = cb;
  num_callbacks++;
}

uint16_t lora_controller_get_val(uint8_t channel)
{
  uint16_t value;
  assert(status == LORA_CONTROLLER_CONNECTED);
  assert(channel < RC_NUM_CHANNELS);
  value = channel_values[channel];
  return value;
}

static void lora_receive_task(void* args)
{
  while (true) {
    uint32_t i = 0;
    uint32_t notification;
    
    assert(xTaskNotifyWait(LORA_DATA_NOTIFICATION, 0, &notification, portMAX_DELAY));
    assert(notification == LORA_DATA_NOTIFICATION);

    // Needed to set correct state for LoRa module
    // If you get an build error here, then you need to move handleDio0Rise into the public methods in LoRa.h library file.
    LoRa.handleDio0Rise();

    while (LoRa.available()) {
      if (i < LORA_PACKET_LENGTH) {
        receive_buf[i] = (uint8_t)LoRa.read();
      } else {
        LoRa.read();
      }
      i++;
    }

    if (i != LORA_PACKET_LENGTH) {
      printf("ERROR: Unexpected data length: %d\n", i);
    } else {
      memcpy(channel_values, receive_buf, LORA_PACKET_LENGTH);
      //printf("Got: %d, %d \t %d, %d \t %d, %d\n", channel_values[0], channel_values[1], channel_values[2],  channel_values[3], channel_values[4], channel_values[5]);
    }
    attachInterrupt(digitalPinToInterrupt(DIO0), lora_availible_isr, RISING);
  }
}

static IRAM_ATTR void lora_availible_isr(void)
{
  last_lora_data_ticks = xTaskGetTickCountFromISR();
  detachInterrupt(digitalPinToInterrupt(DIO0));
  assert(xTaskNotifyFromISR(task_handle, LORA_DATA_NOTIFICATION, eSetValueWithOverwrite, NULL) == pdPASS);
}

static void lora_state_checker(void* args)
{
  while (true) {
    if (last_lora_data_ticks > last_lora_data_checked_ticks) {
      if (status == LORA_CONTROLLER_DISCONNECTED) {
        status = LORA_CONTROLLER_CONNECTED;
        for (uint8_t i = 0; i < num_callbacks; i++) {
          status_callbacks[i](LORA_CONTROLLER_CONNECTED);
        }
      }
    } else {
      if (status == LORA_CONTROLLER_CONNECTED) {
        status = LORA_CONTROLLER_DISCONNECTED;
        for (uint8_t i = 0; i < num_callbacks; i++) {
          status_callbacks[i](LORA_CONTROLLER_DISCONNECTED);
        }
      }
    }
    last_lora_data_checked_ticks = xTaskGetTickCount();
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}