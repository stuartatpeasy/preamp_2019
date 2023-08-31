/*
    controls.c - reads rotary encoder, balance pot and pushbuttons; uses readings to drive PGA2311
    variable-gain amplifier, relays, channel-indicator LEDs, etc.

    Stuart Wallace <stuartw@atom.net>, February 2019.
*/

#include "platform.h"
#include "controls.h"
#include "expander.h"
#include "ir.h"
#include "laws.h"
#include "nvmem.h"
#include "shiftreg.h"
#include "lib/adc.h"
#include "lib/debug.h"
#include "lib/gpio.h"
#include "lib/spi.h"
#include "util/irq.h"
#include <stdlib.h>
#include <util/delay.h>


// EEPROM addresses at which persistent data are stored
#define EEPROM_ADDR_ENC_POS         ((uint8_t *) 0)     // Encoder position
#define EEPROM_ADDR_CHANNEL         ((uint8_t *) 4)     // Channel number

static void isr_controls();
static void update_buttons();


// In order to de-bounce values read from the rotary encoder, each value is shifted in to the LSB
// position of a uint8_t accumulator; the MSB is discarded.  The encoder is considered stable once
// eight consecutive reads have returned the same value, i.e. the accumulator reaches a value of
// 0x00 or 0xff.
#define ENC_IS_STABLE(x)    (((x) == 0x00) || ((x) == 0xff))

// Minimum and maximum position values for the volume-control encoder.  The encoder can be rotated
// past these values, but such additional rotation is ignored.
#define ENC_POS_MIN         (0)
#define ENC_POS_MAX         (95)

// Rate, in Hz, at which controls are sampled.
#define CONTROL_READ_FREQ   (4000)

// Number of control-read cycles during which control positions must be unchanged before control
// positions are written to EEPROM.
#define EEPROM_WRITE_WAIT   (2 * CONTROL_READ_FREQ)     // 2-second delay

// These variables are used as de-bouncing accumulators for the rotary encoder inputs.
static uint8_t debounce_a, debounce_b;

// The bal_accum variable acts as an accumulator for values read from the ADC connected to the
// balance potentiometer.  It holds a moving sum of the last 256 values read from the ADC.
static int32_t bal_accum;

// The balance variable holds a value, in units of 0.5dB, expressing the left/right channel
// balance.  Positive values indicate greater gain on the left channel; negative values denote
// greater gain on the right channel.  e.g. the value -10 indicates -5dB gain of the left channel
// and 5dB gain on the right channel.
static int8_t balance;

// The position of the volume control rotary encoder is stored in volume_enc_pos.
static uint8_t volume_enc_pos;

// Flag indicating that a volume/balance control position has changed but the change has not yet
// been written to the programmable gain amplifier.
static uint8_t change_pending;

// Counter used to implement a delay following a control change before the change is written to
// EEPROM.  For a write to be initiated, controls must be stable for a fixed period of time.
static uint16_t eeprom_write_countdown;

// Array of press/release debounce states for channel-select buttons
static uint8_t channel_buttons[NUM_CHANNELS];
static uint8_t power_button;

// Variable which holds the current channel number
static uint8_t channel;



// controls_init() - initialise peripherals related to the handling of controls and settings in
// the preamplifier.
//
void controls_init()
{
    uint8_t i;

    gpio_make_input(PIN_VOL_ENC_A);
    gpio_make_input(PIN_VOL_ENC_B);
    gpio_make_input(PIN_PB_POWER);
    gpio_set_pullup(PIN_VOL_ENC_A, 1);
    gpio_set_pullup(PIN_VOL_ENC_B, 1);
    gpio_set_pullup(PIN_PB_POWER, 1);

    // Initialise button states
    power_button = 0;
    for(i = 0; i < NUM_CHANNELS; ++i)
        channel_buttons[i] = 0;

    // Set the LSB (and only the LSB) in the de-bouncing accumulators in order to force eight
    // consecutive identical reads before a stable state is reached.  This prevents glitching on
    // startup.
    debounce_a = 0x01;
    debounce_b = 0x01;

    // Initialise the balance accumulator, which holds a moving sum of the last 256 readings from
    // the ADC connected to the balance potentiometer.
    bal_accum = 0;
    balance = 0;

    change_pending = 0;
    eeprom_write_countdown = 0;

    channel = 0;

    // Read initial volume control rotary encoder position from EEPROM
    volume_enc_pos = nvmem_read_byte(EEPROM_ADDR_ENC_POS);

    // If an invalid value was read from the EEPROM, start the system at the minimum volume level
    if(volume_enc_pos > ENC_POS_MAX)
        volume_enc_pos = 0;

    // Configure timer D
    TCD0_CMPBCLR = F_CPU / 4000;                // Set 250us (4kHz) period
    TCD0_INTCTRL = TCD_OVF_bm;                  // Enable interrupt on overflow
    TCD0_CTRLA = TCD_CLKSEL_SYSCLK_gc;          // Clk src = sys clock

    while(!(TCD0_STATUS & TCD_ENRDY_bm))        // Wait for counter to become ready
        ;

    TCD0_CTRLA |= TCD_ENABLE_bm;                // Enable
}


// ISR triggered at a predetermined frequency in order to read control inputs.
//
ISR(TCD0_OVF_vect)
{
    TCD0_INTFLAGS = 0xff;   // Acknowledge interrupts
    isr_controls();
}


// isr_controls() - read all controls and initiate the appropriate action.  This function is called
// every 250us.
//
static void isr_controls()
{
    static uint8_t enc_state = 0, tick = 0;
    static int32_t bal_smoothed = 0;

    ++tick;

    // An input is considered stable when eight consecutive reads return the same value.
    const uint8_t was_stable = ENC_IS_STABLE(debounce_a) && ENC_IS_STABLE(debounce_b);

    debounce_a = (debounce_a << 1) | (gpio_read(PIN_VOL_ENC_A) ? 1 : 0);
    debounce_b = (debounce_b << 1) | (gpio_read(PIN_VOL_ENC_B) ? 1 : 0);

    // If either input was unstable, and both inputs are now stable, a transition has occurred.
    if(!was_stable && ENC_IS_STABLE(debounce_a) && ENC_IS_STABLE(debounce_b))
    {
        // Convert 2-bit Gray code value in {debounce_b, debounce_a} to binary value in new_val
        const uint8_t new_state = debounce_b ? 3 - (debounce_a & 1) : (debounce_a & 1);

        if(new_state == ((enc_state + 1) & 3))
            volume_up();            // Clockwise
        else if(new_state == ((enc_state - 1) & 3))
            volume_down();          // Anti-clockwise

        enc_state = new_state;
    }

    // Every sixteenth cycle (=4ms), update the state of the channel-select buttons
    if(!(tick & 0x0f))
        update_buttons();

    adc_set_channel(adc_channel_from_gpio(PIN_BAL_POT));

    bal_accum -= bal_accum >> 8;
    bal_accum += adc_convert();

    if(abs(bal_accum - bal_smoothed) > 1024)
    {
        bal_smoothed = bal_accum;
        balance = ((bal_smoothed >> 8) - 512) / 24;
        eeprom_write_countdown = EEPROM_WRITE_WAIT;
        change_pending = 1;
    }

    if(eeprom_write_countdown && !--eeprom_write_countdown)
        nvmem_update_byte(EEPROM_ADDR_ENC_POS, volume_enc_pos);
}


// update_buttons() - read the state of the pushbuttons, perform debouncing.
//
static void update_buttons()
{
    uint8_t button_state = expander_get_buttons();
    int8_t i;

    power_button <<= 1;
    if(!gpio_read(PIN_PB_POWER))
        power_button |= 1;

    for(i = 0; i < NUM_CHANNELS; ++i)
    {
        channel_buttons[i] = (channel_buttons[i] << 1) | (button_state & 1);
        button_state >>= 1;
    }
}


// power_button_pressed() - returns non-zero if the power button is pressed; zero otherwise.
// Note that zero will be returned if the power button is in an indeterminate state, i.e. during
// debouncing.
//
uint8_t power_button_pressed()
{
    return power_button == 0xff;
}


// power_button_released() - returns non-zero if the power button is pressed; zero otherwise.
// Note that zero will be returned if the power button is in an indeterminate state, i.e. during
// debouncing.
//
uint8_t power_button_released()
{
    return power_button == 0x00;
}


// volume_up() - if the current volume level is below the maximum, increase output volume level by
// one step and set the "change pending" flag so that the new volume level will be applied next
// time the volume_apply_change() function is called.
//
void volume_up()
{
    if(volume_enc_pos < ENC_POS_MAX)
    {
        ++volume_enc_pos;
        change_pending = 1;
        eeprom_write_countdown = EEPROM_WRITE_WAIT;
    }
}


// volume_down() - if the current volume level is above the minimum, decrease output volume level
// by one step and set the "change pending" flag so that the new volume level will be applied next
// time the volume_apply_change() function is called.
//
void volume_down()
{
    if(volume_enc_pos > 0)
    {
        --volume_enc_pos;
        change_pending = 1;
        eeprom_write_countdown = EEPROM_WRITE_WAIT;
    }
}


// volume_apply_change() - if there have been changes to the output volume levels (as a result of
// a change in the position of the volume or balance controls), as indicated by the change_pending
// flag, apply the new volume levels to the PGA2311 programmable-gain amplifier.
//
void volume_apply_change()
{
    if(change_pending)
    {
        const uint8_t base_val = pga2311_gain_from_enc_pos(volume_enc_pos);
        int16_t left_gain = base_val + balance, right_gain = base_val - balance;

        if(left_gain < 0)
            left_gain = 0;

        if(left_gain > 255)
            left_gain = 255;

        if(right_gain < 0)
            right_gain = 0;

        if(right_gain > 255)
            right_gain = 255;

        interrupt_enable_decrement();

        //gpio_make_output(PIN_PGA_nCS);      // TEMP: assert PGA nCS
        spi0_tx(right_gain);                // Transmit right channel gain value
        spi0_tx(left_gain);                 // Transmit left channel gain value
        spi0_flush_tx();
        //gpio_make_input(PIN_PGA_nCS);       // TEMP: negate PGA nCS

        spi0_read();                        // } Perform two dummy reads to
        spi0_read();                        // } clear the SPI input buffer

        interrupt_enable_increment();

        debug_printf("%d %d\n", left_gain, right_gain);
        change_pending = 0;
    }
}


// set_channel() - switch to the channel specified by <channel>.  If <channel> equals zero, all
// inputs will be disabled.  This function uses various delays to avoid audio thumps and bangs.
//
void set_channel(const uint8_t c)
{
    if(c && (channel == c))
        return;

    shiftreg_enable_output_relay(0);            // Open output relay
    expander_set_leds(0);                       // Switch off previous channel LEDs
    _delay_ms(10);                              // Wait for output relay to open
    shiftreg_set_channel(0);                    // Open old channel relay
    _delay_ms(10);                              // Wait for old channel relay to open

    if(c)
    {
        shiftreg_set_channel(c);                // Close new channel relay
        expander_set_leds(1 << (c - 1));        // Switch on new channel LED
        _delay_ms(200);                         // Wait for analogue section to settle
        shiftreg_enable_output_relay(1);        // Close output relay

        nvmem_update_byte(EEPROM_ADDR_CHANNEL, c);
    }

    channel = c;
}


// set_channel_from_nv() - switch to the channel whose number is stored in non-volatile memory.
// This function is called after startup to recall and set the last-selected input channel.
//
void set_channel_from_nv()
{
    uint8_t channel;

    channel = nvmem_read_byte(EEPROM_ADDR_CHANNEL);

    if(!channel || (channel > NUM_CHANNELS))
    {
        channel = 1;
        nvmem_update_byte(EEPROM_ADDR_CHANNEL, channel);
    }

    set_channel(channel);
}


// get_command() -
//
Command_t get_command()
{
    Command_t cmd = ir_get_cmd();

    // Commands received via infrared have a higher priority than command inputs from buttons.
    // If no infrared command has been received, check for command inputs from buttons.
    if(cmd == CmdNone)
    {
        uint8_t channel;

        if(power_button_pressed())
            cmd = CmdPower;
        else
        {
            for(channel = 0; channel < NUM_CHANNELS; ++channel)
                if(channel_buttons[channel] == 0xff)
                    cmd = CmdChannel1 + channel;
        }
    }

    return cmd;
}