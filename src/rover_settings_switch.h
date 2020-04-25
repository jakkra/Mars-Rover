#pragma once

typedef enum RoverSwitchState {
    ROVER_SWITCH_STATE_STATION_LORA,
    ROVER_SWITCH_STATE_LORA_ONLY,
    ROVER_SWITCH_STATE_AP,
} RoverSwitchState;

void rover_settings_switch_init(void);
RoverSwitchState rover_settings_switch_get_state(void);