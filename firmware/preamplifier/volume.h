#ifndef VOLUME_H_INC
#define VOLUME_H_INC
/*
    volume.h - reads rotary encoder and balance pot; uses readings to drive PGA2311 variable-gain
    amplifier.

    Stuart Wallace <stuartw@atom.net>, February 2019.
*/

void volume_init();
void volume();

#endif
