#ifndef SHIFTREG_H_INC
#define SHIFTREG_H_INC
/*
    shiftreg.h - contains functions which perform various operations through the TPIC6B595 shift
    register / open-drain driver IC.  This is not a generic driver for the TPIC6B595.

    Stuart Wallace <stuartw@atom.net>, March 2019.
*/

#include <stdint.h>


void shiftreg_init();
void shiftreg_set_channel(uint8_t channel);
void shiftreg_enable_output_relay(const uint8_t enable);

#endif
