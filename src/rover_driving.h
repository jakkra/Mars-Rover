#pragma once

#include "switch_checker.h"

void rover_driving_init(void);
void rover_driving_set_drive_mode(RoverMode mode);
void rover_driving_move(uint16_t signal);
void rover_driving_steer(uint16_t signal);