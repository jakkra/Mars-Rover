#include "rover_settings_switch.h"
#include "driver/gpio.h"
#include "rover_config.h"

void rover_settings_switch_init(void)
{
    gpio_pad_select_gpio((gpio_num_t)ROVER_SETTINGS_SWITCH_1);
    gpio_set_direction((gpio_num_t)ROVER_SETTINGS_SWITCH_1, GPIO_MODE_INPUT);
    gpio_set_pull_mode((gpio_num_t)ROVER_SETTINGS_SWITCH_1, GPIO_PULLDOWN_ONLY);

    gpio_pad_select_gpio((gpio_num_t)ROVER_SETTINGS_SWITCH_2);
    gpio_set_direction((gpio_num_t)ROVER_SETTINGS_SWITCH_2, GPIO_MODE_INPUT);
    gpio_set_pull_mode((gpio_num_t)ROVER_SETTINGS_SWITCH_2, GPIO_PULLDOWN_ONLY);
}

RoverSwitchState rover_settings_switch_get_state(void)
{
    RoverSwitchState state = ROVER_SWITCH_STATE_STATION_LORA;
    uint32_t sw1 = gpio_get_level((gpio_num_t)ROVER_SETTINGS_SWITCH_1);
    uint32_t sw2 = gpio_get_level((gpio_num_t)ROVER_SETTINGS_SWITCH_2);

    if (sw1 > 0) {
        state = ROVER_SWITCH_STATE_STATION_LORA;
    } else if (sw2 > 0) {
        state = ROVER_SWITCH_STATE_AP;
    } else {
        state = ROVER_SWITCH_STATE_LORA_ONLY;
    }

    return state;
}