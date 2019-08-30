#ifndef IR_H_INC
#define IR_H_INC
/*
    ir.h - receives infrared remote control commands using the RC-5 protocol

    Stuart Wallace <stuartw@atom.net>, March 2019.
*/

#include "commands.h"
#include <stdint.h>


void ir_init();
Command_t ir_get_cmd();

#endif
