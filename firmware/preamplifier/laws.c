/*
    laws.c - contains data specifying the relationship between control position and gain-control
    values.  These lookup tables enable volume/balance control positions to be mapped to values
    which set the gain of the PGA2311 programmable-gain amplifier.

    Stuart Wallace <stuartw@atom.net>, March 2019.
*/

#include "laws.h"
#include <avr/pgmspace.h>       // The tables are stored in Flash


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
// The array is padded to 128 elements in order to simplify argument-validation code in the
// pga2311_gain_from_enc_pos() function.
//
static const uint8_t volume_law[128] PROGMEM =
{
//  N          Gain/dB
    0,      // -96.0
    0,      // -96.0
    24,     // -84.0
    45,     // -73.5
    60,     // -66.0
    71,     // -60.5
    80,     // -56.0
    88,     // -52.0
    95,     // -48.5
    101,    // -45.5
    106,    // -43.0
    111,    // -40.5
    116,    // -38.0
    120,    // -36.0
    123,    // -34.5
    127,    // -32.5
    130,    // -31.0
    133,    // -29.5
    136,    // -28.0
    139,    // -26.5
    141,    // -25.5
    143,    // -24.5
    146,    // -23.0
    148,    // -22.0
    150,    // -21.0
    152,    // -20.0
    154,    // -19.0
    156,    // -18.0
    157,    // -17.5
    159,    // -16.5
    161,    // -15.5
    162,    // -15.0
    164,    // -14.0
    165,    // -13.5
    167,    // -12.5
    168,    // -12.0
    169,    // -11.5
    170,    // -11.0
    172,    // -10.0
    173,    //  -9.5
    174,    //  -9.0
    175,    //  -8.5
    176,    //  -8.0
    177,    //  -7.5
    178,    //  -7.0
    179,    //  -6.5
    180,    //  -6.0
    181,    //  -5.5
    182,    //  -5.0
    183,    //  -4.5
    184,    //  -4.0
    185,    //  -3.5
    186,    //  -3.0
    187,    //  -2.5
    188,    //  -2.0
    188,    //  -2.0
    189,    //  -1.5
    190,    //  -1.0
    191,    //  -0.5
    191,    //  -0.5
    192,    //   0.0
    193,    //   0.5
    194,    //   1.0
    194,    //   1.0
    195,    //   1.5
    196,    //   2.0
    196,    //   2.0
    197,    //   2.5
    198,    //   3.0
    198,    //   3.0
    199,    //   3.5
    199,    //   3.5
    200,    //   4.0
    201,    //   4.5
    201,    //   4.5
    202,    //   5.0
    202,    //   5.0
    203,    //   5.5
    203,    //   5.5
    204,    //   6.0
    204,    //   6.0
    205,    //   6.5
    205,    //   6.5
    206,    //   7.0
    206,    //   7.0
    207,    //   7.5
    207,    //   7.5
    208,    //   8.0
    208,    //   8.0
    209,    //   8.5
    209,    //   8.5
    209,    //   8.5
    210,    //   9.0
    210,    //   9.0
    211,    //   9.5
    211,    //   9.5
    0,      // Padding
    0,      // Padding
    0,      // Padding
    0,      // Padding
    0,      // Padding
    0,      // Padding
    0,      // Padding
    0,      // Padding
    0,      // Padding
    0,      // Padding
    0,      // Padding
    0,      // Padding
    0,      // Padding
    0,      // Padding
    0,      // Padding
    0,      // Padding
    0,      // Padding
    0,      // Padding
    0,      // Padding
    0,      // Padding
    0,      // Padding
    0,      // Padding
    0,      // Padding
    0,      // Padding
    0,      // Padding
    0,      // Padding
    0,      // Padding
    0,      // Padding
    0,      // Padding
    0,      // Padding
    0,      // Padding
    0       // Padding
};



uint8_t pga2311_gain_from_enc_pos(const uint8_t enc_pos)
{
    return pgm_read_byte(&volume_law[enc_pos & 0x7f]);
};
