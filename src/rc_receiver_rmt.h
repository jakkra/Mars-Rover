#define RECEIVER_CH_NUM 6
#define RECEIVER_CH_CENTER 1500

void rc_receiver_rmt_init(void);
uint16_t rc_receiver_rmt_get_val(uint8_t channel);