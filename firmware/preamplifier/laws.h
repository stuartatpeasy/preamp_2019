#ifndef LAWS_H_INC
#define LAWS_H_INC
/*
    laws.h - contains data specifying the relationship between control position and gain-control
    values.  These lookup tables enable volume/balance control positions to be mapped to values
    which set the gain of the PGA2311 programmable-gain amplifier.

    Stuart Wallace <stuartw@atom.net>, March 2019.
*/

#include <stdint.h>


uint8_t pga2311_gain_from_enc_pos(const uint8_t enc_pos);

#endif
