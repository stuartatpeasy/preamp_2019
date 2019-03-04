/*
    main.c - entry point

    Stuart Wallace <stuartw@atom.net>, September 2018.
*/

#include "ir.h"
#include "platform.h"           // for MODULE_TYPE
#include "lib/adc.h"
#include "lib/clk.h"
#include "lib/debug.h"
#include "lib/gpio.h"
#include "lib/vref.h"
#include "volume.h"
#include <avr/interrupt.h>      // sei()
#include <util/delay.h>


void firmware_main(void);

int8_t g_irq_state = 0;     // Used in util/irq.h


// Entry point
//
int main(void)
{
    pclk_set_divisor_val(2);                // Set peripheral clock = main clock / 2
    pclk_enable();                          // Enable peripheral clock

    debug_init();
    debug_putstr_p("SW Preamplifier mk0 firmware\n");

    vref_set(VRefADC0, VRef2V5);
    vref_enable(VRefADC0, 1);

    adc_set_vref(ADCRefInternal, 1);
    adc_set_prescaler(ADCPrescaleDiv8);
    adc_set_initdelay(ADCInitDelay16);
    adc_enable(1);


    // firmware_main() should loop eternally; loop here in case it doesn't.
    while(1)
        firmware_main();        // should never return
}



void firmware_main()
{
    volume_init();
    ir_init();

    sei();

    while(1)
    {
        volume();
        _delay_us(250);
    }
}
