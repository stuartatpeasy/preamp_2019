#ifndef COMMANDS_H_INC
#define COMMANDS_H_INC
/*
    commands.h - definitions of codes representing commands received via infrared or physical
    control inputs.
*/


// Command codes - these must be in the range 0x00-0x3f for compatibility with the RC-5 protocol.
//
typedef enum Command
{
    CmdNone         = 0x00,         // No command
    CmdPower        = 0x01,         // Power button
    CmdMute         = 0x02,         // Mute button
    CmdChannel1     = 0x10,         // Channel 1
    CmdChannel2     = 0x11,         // Channel 2
    CmdChannel3     = 0x12,         // Channel 3
    CmdChannel4     = 0x13,         // Channel 4
    CmdVolUp        = 0x20,         // Increase volume
    CmdVolDown      = 0x21          // Decrease volume
} Command_t;

#endif
