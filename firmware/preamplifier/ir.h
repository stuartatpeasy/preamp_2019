#ifndef IR_H_INC
#define IR_H_INC
/*
    ir.h - receives infrared remote control commands using the RC-5 protocol

    Stuart Wallace <stuartw@atom.net>, March 2019.
*/

#include <stdint.h>

// IRCommand_t - encapsulates an RC-5 packet
typedef uint16_t IRCommand_t;

// Extract the address field (bits 6-10) from an RC-5 packet
#define IR_ADDRESS(x)       (((x) >> 6) & 0x1f)

// Extract the command field (bits 0-5) from an RC-5 packet
#define IR_COMMAND(x)       ((x) & 0x3f)

// Evaluate to non-zero if the "toggle" bit (bit 11) is set; zero otherwise
#define IR_TOGGLE(x)        ((x) & 0x800)


void ir_init();
IRCommand_t ir_get_cmd();


#endif
