#ifndef PLATFORM_H_INC
#define PLATFORM_H_INC
/*
    platform.h: declarations relevant to both main.c and other modules

    Stuart Wallace <stuartw@atom.net>, July 2018.

    Note that the pinout declarations in this file make use of the GPIO*() macros defined in
    lib/gpio.h.
*/

// Uncomment this to enable the debug_printf() function; otherwise this function is a NOP.
#define DEBUG_WITH_PRINTF

// Note that a macro, defining the module type for which we're building firmware, must be defined
// somewhere.  This definition currently resides in the project configuration.  Before build, a
// specific configuration is selected: e.g. "PA_MONO_TAS5760M_Debug", for a debug build of firmware
// for a TAS5760M-based mono PA module.  Macros to configure this build are then defined under
// Project -> Properties... -> AVR/GNU C Compiler -> Symbols.


//
// Definitions common to all module types
//

// The preamplifier has no device-specific builds; it always uses ATTiny816
#define WITH_ATTINY816

#define F_CPU           10000000UL          // Scaled CPU frequency in Hz
#define F_EXT_CLK       0UL                 // uC ext clock frequency; 0 if no ext clock is used

#define NUM_CHANNELS    4                   // Number of input channels, including Bluetooth

//
// Per-module pin-out, compilation and configuration directives
//
/*
                           ATtiny816
                       +---------------+
                  VDD -| 1          20 |- GND
                  PA4 -| 2          19 |- PA3  SPI_SCK
       VMI_SENSE  PA5 -| 3          18 |- PA2  SPI_MISO
       VPL_SENSE  PA6 -| 4          17 |- PA1  SPI_MOSI
         BAL_POT  PA7 -| 5          16 |- PA0  UPDI
        PB_POWER  PB5 -| 6          15 |- PC3  SR_RCLK
          REG_EN  PB4 -| 7          14 |- PC2  EXP_nCS
         UART_RX  PB3 -| 8          13 |- PC1  EXP_nRESET
         UART_TX  PB2 -| 9          12 |- PC0  IR_RX
       VOL_ENC_A  PB1 -| 10         11 |- PB0  VOL_ENC_B
                       +---------------+
*/

// Port A
#define PIN_BAL_POT                 GPIOA(7)        // [AI] Balance pot wiper
#define PIN_VPL_SENSE               GPIOA(6)        // [AI] Vplus (+15V) sense
#define PIN_VMI_SENSE               GPIOA(5)        // [AI] Vminus (-15V) sense (indirect)

// Port B
#define PIN_PB_POWER                GPIOB(5)        // [I] Power pushbutton input
#define PIN_REG_EN                  GPIOB(4)        // [O] Enable signal for voltage regulators
#define PIN_VOL_ENC_A               GPIOB(1)        // [I] Volume rotary encoder, pin B
#define PIN_VOL_ENC_B               GPIOB(0)        // [I] Volume rotary encoder, pin A

// Port C
#define PIN_SR_RCLK                 GPIOC(3)        // [O] TPIB6B595 register clock
#define PIN_EXP_nCS                 GPIOC(2)        // [O] MCP23S17 port expander chip select
#define PIN_EXP_nRESET              GPIOC(1)        // [O] MCP23S17 port expander reset
#define PIN_IR_RX                   GPIOC(0)        // [I] Infrared demodulator input

#endif      // PLATFORM_H_INC

#if defined(WITH_ATTINY816)
#include "platform_attinyX16.h"
#elif defined(WITH_ATTINY814)
#include "platform_attinyX14.h"
#endif
