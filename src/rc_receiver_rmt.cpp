#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt.h"
#include "esp_log.h"
#include "config.h"
#include "rover_config.h"
#include "rc_receiver_rmt.h"


#define RMT_CLK_DIV      10
#define RMT_TICK_US    (80000000/RMT_CLK_DIV/1000000)
#define PPM_IMEOUT_US  3500 // RMT receiver timeout value(us)

#define RMT_TICK_PER_US 8
#define RMT_RX_CLK_DIV (80000000/RMT_TICK_PER_US/1000000)
#define RMT_RX_MAX_US 3500

static const char* TAG = "RC_RMT";

static volatile uint16_t channel_values[RC_NUM_CHANNELS] = {0};
static const uint8_t RECEIVER_CHANNELS[RC_NUM_CHANNELS] = { 1, 2, 3, 4, 5, 6 };
static const uint8_t RECEIVER_GPIOS[RC_NUM_CHANNELS] = { RC_CH1_INPUT, RC_CH2_INPUT, RC_CH3_INPUT, RC_CH4_INPUT, RC_CH5_INPUT, RC_CH6_INPUT };


static void rmt_isr_handler(void* arg){
  uint32_t intr_st = RMT.int_st.val;
  uint8_t i;

  for (i = 0; i < RC_NUM_CHANNELS; i++){
    uint8_t channel = RECEIVER_CHANNELS[i];
    uint32_t channel_mask = BIT(channel * 3 + 1);

    if (!(intr_st & channel_mask)) continue;

    RMT.conf_ch[channel].conf1.rx_en = 0;
    RMT.conf_ch[channel].conf1.mem_owner = RMT_MEM_OWNER_TX;
    volatile rmt_item32_t* item = RMTMEM.chan[channel].data32;
    if (item) {
        channel_values[i] = item->duration0 / RMT_TICK_PER_US; // Convert to 1000-2000 us
    }

    RMT.conf_ch[channel].conf1.mem_wr_rst = 1;
    RMT.conf_ch[channel].conf1.mem_owner = RMT_MEM_OWNER_RX;
    RMT.conf_ch[channel].conf1.rx_en = 1;

    // Clear RMT interrupt status
    RMT.int_clr.val = channel_mask;
  }
}

void rc_receiver_rmt_init(void)
{
  uint8_t i;

  rmt_config_t rmt_channels[RC_NUM_CHANNELS] = {};

  for (i = 0; i < RC_NUM_CHANNELS; i++) {
    channel_values[i] = RECEIVER_CH_CENTER;

    rmt_channels[i].channel = (rmt_channel_t) RECEIVER_CHANNELS[i];
    rmt_channels[i].gpio_num = (gpio_num_t) RECEIVER_GPIOS[i];
    rmt_channels[i].clk_div = RMT_RX_CLK_DIV;
    rmt_channels[i].mem_block_num = 1;
    rmt_channels[i].rmt_mode = RMT_MODE_RX;
    rmt_channels[i].rx_config.filter_en = true;
    rmt_channels[i].rx_config.filter_ticks_thresh = 100;
    rmt_channels[i].rx_config.idle_threshold = RMT_RX_MAX_US * RMT_TICK_PER_US;

    rmt_config(&rmt_channels[i]);
    rmt_set_rx_intr_en(rmt_channels[i].channel, true);
    rmt_rx_start(rmt_channels[i].channel, 1);
  }

  rmt_isr_register(rmt_isr_handler, NULL, 0, NULL);
  ESP_LOGI(TAG, "Init ISR on %d", xPortGetCoreID());
}

uint16_t rc_receiver_rmt_get_val(uint8_t channel)
{
  assert(channel < RC_NUM_CHANNELS);
  return channel_values[channel];    
}