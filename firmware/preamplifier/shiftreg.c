/*
    shiftreg.c - contains functions which perform various operations through the TPIC6B595 shift
    register / open-drain driver IC.  This is not a generic driver for the TPIC6B595.

    Stuart Wallace <stuartw@atom.net>, March 2019.
*/

#include "platform.h"
#include "shiftreg.h"
#include "expander.h"
#include "lib/debug.h"
#include "lib/gpio.h"
#include "lib/spi.h"
#include "util/irq.h"


#define SHIFTREG_OUTPUT_RELAY_BIT   (0x80)  // Shift register output connected to output relay

static uint8_t sr_val;                      // Holds the current value of the shift register

static void shiftreg_update_value();


// sr_init() - initialise the TPIC6B595 shift register / driver by disabling its outputs and
// setting its register to zero.
//
void shiftreg_init()
{
    gpio_clear(PIN_SR_RCLK);
    gpio_make_output(PIN_SR_RCLK);

    expander_sr_output_enable(0);
    sr_val = 0;
    shiftreg_update_value();
}


// shiftreg_set_channel() - activate the shift register output corresponding to the channel
// specified in <channel>.  This has the effect of activating a set of relays associated with an
// input channel, and deactivating all other sets of input relays.  If <channel> equals zero, all
// input-channel relay controls will be deactivated.
//
void shiftreg_set_channel(uint8_t channel)
{
    if(channel > NUM_CHANNELS)
        channel = 0;

    sr_val &= ~((1 << NUM_CHANNELS) - 1);
    if(channel)
        sr_val |= 1 << (channel - 1);

    shiftreg_update_value();
}


// shiftreg_enable_output_relay() - switch on (if <enable> is non-zero) or off (if <enable> equals
// zero) the audio output relay.
//
void shiftreg_enable_output_relay(const uint8_t enable)
{
    if(enable)
        sr_val |= SHIFTREG_OUTPUT_RELAY_BIT;
    else
        sr_val &= ~SHIFTREG_OUTPUT_RELAY_BIT;

    shiftreg_update_value();
}


// shiftreg_update_value() - update the contents of the shift register so that it matches the value
// of the global variable <sr_val>.
//
static void shiftreg_update_value()
{
    interrupt_enable_decrement();
    spi0_tx(sr_val);
    spi0_flush_tx();
    spi0_read();            // Dummy read
    gpio_set(PIN_SR_RCLK);
    gpio_clear(PIN_SR_RCLK);
    interrupt_enable_increment();
}
