/*
    controls.c - reads rotary encoder, balance pot and pushbuttons; uses readings to drive PGA2311
    variable-gain amplifier, relays, channel-indicator LEDs, etc.

    Stuart Wallace <stuartw@atom.net>, February 2019.
*/

#include "platform.h"
#include "controls.h"
#include "lib/adc.h"
#include "lib/debug.h"
#include "lib/gpio.h"
#include "lib/spi.h"
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdlib.h>


static void isr_controls();


// This array maps the 96 possible values, representing distinct volume-control settings, to gain
// settings used by the PGA2311 programmable-gain amplifier.  The volume-control law uses this
// equation:
//
// gain = 20 log ((1 - (1.2 ^ -(X / 30))) ^ 3) + 31
//
// where X represents the volume-control position, and can take integer values in the range 0-96.
// The equation shown is intended to compress gain at the bottom of the volume control range, and
// expand it at the top of the range.  The values and coefficients in the equation were chosen by
// hand, while rendering a gain-vs-volume-control-position graph; they have no special
// significance.
//
static const uint8_t vol_ctrl_map[96] PROGMEM =
{
//    N          Gain/dB
      0,      // -96.0
      0,      // -96.0
     24,      // -84.0
     45,      // -73.5
     60,      // -66.0
     71,      // -60.5
     80,      // -56.0
     88,      // -52.0
     95,      // -48.5
    101,      // -45.5
    106,      // -43.0
    111,      // -40.5
    116,      // -38.0
    120,      // -36.0
    123,      // -34.5
    127,      // -32.5
    130,      // -31.0
    133,      // -29.5
    136,      // -28.0
    139,      // -26.5
    141,      // -25.5
    143,      // -24.5
    146,      // -23.0
    148,      // -22.0
    150,      // -21.0
    152,      // -20.0
    154,      // -19.0
    156,      // -18.0
    157,      // -17.5
    159,      // -16.5
    161,      // -15.5
    162,      // -15.0
    164,      // -14.0
    165,      // -13.5
    167,      // -12.5
    168,      // -12.0
    169,      // -11.5
    170,      // -11.0
    172,      // -10.0
    173,      //  -9.5
    174,      //  -9.0
    175,      //  -8.5
    176,      //  -8.0
    177,      //  -7.5
    178,      //  -7.0
    179,      //  -6.5
    180,      //  -6.0
    181,      //  -5.5
    182,      //  -5.0
    183,      //  -4.5
    184,      //  -4.0
    185,      //  -3.5
    186,      //  -3.0
    187,      //  -2.5
    188,      //  -2.0
    188,      //  -2.0
    189,      //  -1.5
    190,      //  -1.0
    191,      //  -0.5
    191,      //  -0.5
    192,      //   0.0
    193,      //   0.5
    194,      //   1.0
    194,      //   1.0
    195,      //   1.5
    196,      //   2.0
    196,      //   2.0
    197,      //   2.5
    198,      //   3.0
    198,      //   3.0
    199,      //   3.5
    199,      //   3.5
    200,      //   4.0
    201,      //   4.5
    201,      //   4.5
    202,      //   5.0
    202,      //   5.0
    203,      //   5.5
    203,      //   5.5
    204,      //   6.0
    204,      //   6.0
    205,      //   6.5
    205,      //   6.5
    206,      //   7.0
    206,      //   7.0
    207,      //   7.5
    207,      //   7.5
    208,      //   8.0
    208,      //   8.0
    209,      //   8.5
    209,      //   8.5
    209,      //   8.5
    210,      //   9.0
    210,      //   9.0
    211,      //   9.5
    211       //   9.5
};

// In order to de-bounce values read from the rotary encoder, each value is shifted in to the LSB
// position of a uint8_t accumulator; the MSB is discarded.  The encoder is considered stable once
// eight consecutive reads have returned the same value, i.e. the accumulator reaches a value of
// 0x00 or 0xff.
#define ENC_IS_STABLE(x)    (((x) == 0x00) || ((x) == 0xff))

// Minimum and maximum position values for the volume-control encoder.  The encoder can be rotated
// past these values, but such additional rotation is ignored.
#define ENC_POS_MIN         (0)
#define ENC_POS_MAX         (95)

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


void controls_init()
{
    gpio_make_output(GPIOA(4));             // Temporary: busy signal

    gpio_make_input(PIN_VOL_ENC_A);
    gpio_make_input(PIN_VOL_ENC_B);
    gpio_make_output(PIN_PGA_nCS);          // TEMP: PGA nCS pin: only assert; tri-state while negated
    gpio_clear(PIN_PGA_nCS);                // TEMP: PGA nCS pin: only assert
    gpio_set_pullup(PIN_VOL_ENC_A, 1);
    gpio_set_pullup(PIN_VOL_ENC_B, 1);

    // Set the LSB (and only the LSB) in the de-bouncing accumulators in order to force eight
    // consecutive identical reads before a stable state is reached.  This prevents glitching on
    // startup.
    debounce_a = 0x01;
    debounce_b = 0x01;

    // Initialise the balance accumulator, which holds a moving sum of the last 256 readings from
    // the ADC connected to the balance potentiometer.
    bal_accum = 0;
    balance = 0;

    // TODO: read initial volume control rotary encoder position from EEPROM
    volume_enc_pos = 0;

    // Configure timer D
    TCD0_CMPASET = 0;
    TCD0_CMPACLR = 1;
    TCD0_CMPBSET = 2;
    TCD0_CMPBCLR = F_CPU / 4000;                // Set 250ns (4kHz) period
    TCD0_INTCTRL = TCD_OVF_bm;                  // Enable interrupt on overflow
    TCD0_CTRLA = TCD_CLKSEL_SYSCLK_gc;          // Clk src = sys clock

    while(!(TCD0_STATUS & TCD_ENRDY_bm))        // Wait for counter to become ready
        ;

    TCD0_CTRLA |= TCD_ENABLE_bm;                // Enable
}


ISR(TCD0_OVF_vect)
{
    isr_controls();
    TCD0_INTFLAGS = 0xff;   // Acknowledge interrupts
}


static void isr_controls()
{
    static uint8_t enc_state = 0;
    static int32_t bal_smoothed = 0;
    uint8_t changed = 0;

    gpio_set(GPIOA(4));     // Start busy

    // An input is considered stable when eight consecutive reads return the same value.
    const uint8_t was_stable = ENC_IS_STABLE(debounce_a) && ENC_IS_STABLE(debounce_b);

    debounce_a = (debounce_a << 1) | (gpio_read(PIN_VOL_ENC_A) ? 1 : 0);
    debounce_b = (debounce_b << 1) | (gpio_read(PIN_VOL_ENC_B) ? 1 : 0);

    // If either input was unstable, and both inputs are now stable, a transition has occurred.
    if(!was_stable && ENC_IS_STABLE(debounce_a) && ENC_IS_STABLE(debounce_b))
    {
        // Convert 2-bit Gray code value in {debounce_b, debounce_a} to binary value in new_val
        const uint8_t new_state = debounce_b ? 3 - (debounce_a & 1) : (debounce_a & 1);

        if((new_state == ((enc_state + 1) & 3)) && (volume_enc_pos < ENC_POS_MAX))
        {
            ++volume_enc_pos;       // Clockwise
            changed = 1;
        }
        else if((new_state == ((enc_state - 1) & 3)) && (volume_enc_pos > ENC_POS_MIN))
        {
            --volume_enc_pos;       // Anti-clockwise
            changed = 1;
        }

        enc_state = new_state;
    }

    adc_set_channel(adc_channel_from_gpio(PIN_BAL_POT));

    bal_accum -= bal_accum >> 8;
    bal_accum += adc_convert();

    if(abs(bal_accum - bal_smoothed) > 1024)
    {
        bal_smoothed = bal_accum;
        balance = ((bal_smoothed >> 8) - 512) / 24;
        changed = 1;
    }

    if(changed)
    {
        const uint8_t base_val = pgm_read_byte(&vol_ctrl_map[volume_enc_pos]);
        int16_t left_gain = base_val + balance, right_gain = base_val - balance;

        if(left_gain < 0)
            left_gain = 0;

        if(left_gain > 255)
            left_gain = 255;

        if(right_gain < 0)
            right_gain = 0;

        if(right_gain > 255)
            right_gain = 255;


//        gpio_make_output(PIN_PGA_nCS);      // TEMP: assert PGA nCS
        spi0_tx(right_gain);                // Transmit right channel gain value
        spi0_tx(left_gain);                 // Transmit left channel gain value
//        gpio_make_input(PIN_PGA_nCS);       // TEMP: negate PGA nCS

        spi0_read();                        // } Perform two dummy reads to
        spi0_read();                        // } clear the SPI input buffer

        debug_printf("%d %d\n", left_gain, right_gain);
    }

    gpio_clear(GPIOA(4));   // End busy
}
