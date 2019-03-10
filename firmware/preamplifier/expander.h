#ifndef EXPANDER_H_INC
#define EXPANDER_H_INC
/*
    expander.c - contains functions which perform various operations through the MCP23S17 IO port
    expander.  This is not a generic driver for the MCP23S17.

    Stuart Wallace <stuartw@atom.net>, March 2019.
*/

#include <stdint.h>


void expander_init();
void expander_reset();
void expander_set_leds(const uint8_t leds);
uint8_t expander_get_buttons();
void expander_sr_output_enable(const uint8_t enable);

#endif
