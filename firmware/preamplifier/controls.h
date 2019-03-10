#ifndef CONTROLS_H_INC
#define CONTROLS_H_INC
/*
    controls.h - reads rotary encoder, balance pot and pushbuttons; uses readings to drive PGA2311
    variable-gain amplifier, relays, channel-indicator LEDs, etc.

    Stuart Wallace <stuartw@atom.net>, February 2019.
*/

#include "commands.h"
#include <stdint.h>


void controls_init();
void volume_up();
void volume_down();
void volume_apply_change();
uint8_t power_button_pressed();
uint8_t power_button_released();
void set_channel(const uint8_t channel);
void set_channel_from_nv();
Command_t get_command();

#endif
