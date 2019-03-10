/*
    expander.c - contains functions which perform various operations through the MCP23S17 IO port
    expander.  This is not a generic driver for the MCP23S17.

    Stuart Wallace <stuartw@atom.net>, March 2019.
*/

#include "platform.h"
#include "expander.h"
#include "lib/debug.h"
#include "lib/gpio.h"
#include "lib/spi.h"
#include "util/irq.h"
#include <avr/cpufunc.h>


// Macros used when constructing the first byte of a MCP23S17 command packet
//
#define EXPANDER_OPCODE     (0x40)          // Opcode used to indicate an MCP23S17 command
#define EXPANDER_ADDR       (0x00)          // Three-bit address of the MCP23S17 expander
#define EXPANDER_OP_WRITE   (0)             // Indicates a write operation
#define EXPANDER_OP_READ    (1)             // Indicates a read operation


// Pin definitions
#define GPIOA_SR_RCLK       (1 << 7)        // TPIC6B595 register clock
#define GPIOA_SR_nGATE      (1 << 6)        // TPIC6B595 gate (active low)


// MCP23S17Reg_t - enumeration of registers in the MCP23S17 IO port expander
//
typedef enum MCP23S17Reg
{
    MCP23S17Reg_IODIRA          = 0x00,     // [RW] Port A IO direction register
    MCP23S17Reg_IODIRB          = 0x01,     // [RW] Port B IO direction register
    MCP23S17Reg_IPOLA           = 0x02,     // [RW] Port A input port polarity register
    MCP23S17Reg_IPOLB           = 0x03,     // [RW] Port B input port polarity register
    MCP23S17Reg_GPINTENA        = 0x04,     // [RW] Port A interrupt-on-change control register
    MCP23S17Reg_GPINTENB        = 0x05,     // [RW] Port B interrupt-on-change control register
    MCP23S17Reg_DEFVALA         = 0x06,     // [RW] Port A default value register
    MCP23S17Reg_DEFVALB         = 0x07,     // [RW] Port B default value register
    MCP23S17Reg_INTCONA         = 0x08,     // [RW] Port A interrupt control register
    MCP23S17Reg_INTCONB         = 0x09,     // [RW] Port B interrupt control register
    MCP23S17Reg_IOCON           = 0x0a,     // [RW] Port A configuration register
    MCP23S17Reg_GPPUA           = 0x0c,     // [RW] Port A GPIO pull-up resistor control register
    MCP23S17Reg_GPPUB           = 0x0d,     // [RW] Port B GPIO pull-up resistor control register
    MCP23S17Reg_INTFA           = 0x0e,     // [R ] Port A interrupt flag register
    MCP23S17Reg_INTFB           = 0x0f,     // [R ] Port B interrupt flag register
    MCP23S17Reg_INTCAPA         = 0x10,     // [R ] Port A interrupt captured register
    MCP23S17Reg_INTCAPB         = 0x11,     // [R ] Port B interrupt captured register
    MCP23S17Reg_GPIOA           = 0x12,     // [RW] Port A general purpose IO port register
    MCP23S17Reg_GPIOB           = 0x13,     // [RW] Port B general purpose IO port register
    MCP23S17Reg_OLATA           = 0x14,     // [RW] Port A output latch register
    MCP23S17Reg_OLATB           = 0x15      // [RW] Port B output latch register
} MCP23S17Reg_t;


static void expander_write(const MCP23S17Reg_t reg, const uint8_t data);
static uint8_t expander_read(const MCP23S17Reg_t reg);


// expander_write() - write the value specified by <data> to the MCP23S17 register specified by
// <reg>.
//
static void expander_write(const MCP23S17Reg_t reg, const uint8_t data)
{
    interrupt_enable_decrement();
    gpio_clear(PIN_EXP_nCS);
    spi0_tx(EXPANDER_OPCODE | EXPANDER_ADDR | EXPANDER_OP_WRITE);
    spi0_tx(reg);
    spi0_flush_tx();
    spi0_tx(data);
    spi0_read();        // Dummy read
    spi0_read();        // Dummy read
    spi0_flush_tx();
    spi0_read();        // Dummy read
    gpio_set(PIN_EXP_nCS);
    interrupt_enable_increment();
}


// expander_read() - read the value of the MCP23S17 register specified by <reg> and return it.
//
static uint8_t expander_read(const MCP23S17Reg_t reg)
{
    uint8_t ret;

    interrupt_enable_decrement();
    gpio_clear(PIN_EXP_nCS);
    spi0_tx(EXPANDER_OPCODE | EXPANDER_ADDR | EXPANDER_OP_READ);
    spi0_tx(reg);
    spi0_flush_tx();
    spi0_read();        // Dummy read
    spi0_tx(0x00);      // Dummy write
    spi0_read();        // Dummy read
    spi0_flush_tx();
    gpio_set(PIN_EXP_nCS);
    ret = spi0_read();
    interrupt_enable_increment();

    return ret;
}


// expander_reset() - reset the MCP23S17 port expander by asserting its nRESET input, waiting for
// "a while", and negating the nRESET input.  The minimum width of the nRESET input is specified as
// 1us; a loop containing 20 NOP operations will result in a pulse considerably longer than this at
// any realistic CPU core frequency.
//
void expander_reset()
{
    uint8_t i;
    gpio_clear(PIN_EXP_nRESET);

    for(i = 0; i < 20; ++i)
        _NOP();

    gpio_set(PIN_EXP_nRESET);
}


// expander_init() -
//
void expander_init()
{
    // Configure control port pins for the MCP23S17.  It requires an active-low chip-select input
    // and can optionally use an active-low reset input.
    gpio_set(PIN_EXP_nCS);
    gpio_set(PIN_EXP_nRESET);
    gpio_make_output(PIN_EXP_nCS);
    gpio_make_output(PIN_EXP_nRESET);

    // Reset the expander so that its internal state is well-defined
    expander_reset();

    // Configure the MSP23S17: configure pins GPB[7..4] as inputs, and enable pull-up; configure
    // pins GPB[3..0] as outputs and set them to logic 0.  Enable inverters on the four inputs, as
    // their default state is high (because of the pull-ups); with the inverters enabled, the
    // inputs - which are connected to pushbuttons - will read 1 when the button is pressed and 0
    // when it is released.
    expander_write(MCP23S17Reg_GPPUB, 0xf0);
    expander_write(MCP23S17Reg_IODIRB, 0xf0);
    expander_write(MCP23S17Reg_IPOLB, 0xf0);

    // Pins GPA7 and GPA6 are outputs connected to the TPIC6B595 shift register:
    //     GPA7 - SR_RCLK [initial state 0]
    //     GPA6 - SR_nGATE [initial state 1]
    // Pins GPA5 and GPA4 are "assert-only" outputs, connected to the PGA2311 programmable-gain
    // amplifier.  They are configured as inputs unless they are asserted, whereupon they become
    // outputs set to logic 0.  These pins are therefore high-impedance in their negated state.
    // This prevents the PGA2311 drawing current through these pins when its power supply is
    // disabled.
    //     GPA5 - PGA_nCS [initial state 1, i.e. input with pull-up]
    //     GPA4 - PGA_nMUTE [initial state 1, i.e. input with pull-up]
    expander_write(MCP23S17Reg_GPPUA, 0x30);
    expander_write(MCP23S17Reg_IODIRA, 0x3f);
}


// expander_set_leds() - illuminate the four LEDs according to the value of the bitmap in bits 0-3
// of <leds>.  Bit 0 corresponds to LED1, bit 1 corresponds to LED2, etc.  If a bit is set, the
// corresponding LED will be illuminated.
//
void expander_set_leds(const uint8_t leds)
{
    expander_write(MCP23S17Reg_GPIOB, leds & 0x0f);
}


// expander_get_buttons() - read the state of the four channel-select pushbuttons and return it as
// a bitmap in bits 0-3.
//
uint8_t expander_get_buttons()
{
    return expander_read(MCP23S17Reg_GPIOB) >> 4;
}


// expander_sr_output_enable() - enable (if <enable> is non-zero) or disable (if <enable> is zero)
// the TPIB6B595 shift register / driver IC by asserting or negating its nGATE input.
//
void expander_sr_output_enable(const uint8_t enable)
{
    uint8_t state = expander_read(MCP23S17Reg_GPIOA);

    state = enable ? (state & ~GPIOA_SR_nGATE) : (state | GPIOA_SR_nGATE);
    expander_write(MCP23S17Reg_GPIOA, state);
}
