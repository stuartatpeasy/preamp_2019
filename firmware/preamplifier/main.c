/*
    main.c - entry point

    Stuart Wallace <stuartw@atom.net>, September 2018.
*/

#include "platform.h"                       // for MODULE_TYPE
#include "controls.h"
#include "ir.h"
#include "lib/adc.h"
#include "lib/clk.h"
#include "lib/debug.h"
#include "lib/gpio.h"
#include "lib/spi.h"
#include "lib/vref.h"
#include <avr/interrupt.h>                  // sei()
#include <stdint.h>
#include <util/delay.h>


#define IR_ADDR_PREAMP      (0x15)          // RC-5 address for infrared commands

#define IR_CMD_POWER        (0x01)          // Power button
#define IR_CMD_MUTE         (0x02)          // Mute button
#define IR_CMD_CHANNEL1     (0x10)          // Channel 1
#define IR_CMD_CHANNEL2     (0x11)          // Channel 2
#define IR_CMD_CHANNEL3     (0x12)          // Channel 3
#define IR_CMD_CHANNEL4     (0x13)          // Channel 4
#define IR_CMD_VOL_UP       (0x20)          // Increase volume
#define IR_CMD_VOL_DOWN     (0x21)          // Decrease volume

void firmware_main(void);

int8_t g_irq_state = 0;                     // Used in util/irq.h


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
    uint8_t power = 0, mute = 0;

    // Initialise SPI port 0 - this is the control interface for the PGA2311 programmable-gain
    // amplifier, the TPIC6B595 shift-register/driver, and the MCP23S17 port expander.
    spi0_configure_master(PinsetDefault, SPIClkDiv4);
    spi0_port_activate(1);
    spi0_enable(1);

    controls_init();
    ir_init();

    sei();

    while(1)
    {
        const IRCommand_t ir_cmd = ir_get_cmd();
        if(ir_cmd && (IR_ADDRESS(ir_cmd) == IR_ADDR_PREAMP))
        {
            if(!power && (IR_COMMAND(ir_cmd) == IR_CMD_POWER))
            {
                power = 1;
                // TODO: handle switch-on
                // Channel switch:
                // - open output relay
                // - switch off previous channel LED
                // - wait 100ms
                // - open previous channel relay
                // - wait 100ms
                // - close new channel relay
                // - illuminate new channel LED
                // - wait 300ms
                // - close output relay
            }
            else
            {
                // TODO: handle IR command...
                switch(IR_COMMAND(ir_cmd))
                {
                    case IR_CMD_POWER:
                        // TODO: handle switch-off
                        power = 0;
                        break;

                    case IR_CMD_MUTE:
                        mute = !mute;
                        // TODO: mute/un-mute
                        break;

                    case IR_CMD_CHANNEL1:
                        break;

                    case IR_CMD_CHANNEL2:
                        break;

                    case IR_CMD_CHANNEL3:
                        break;

                    case IR_CMD_CHANNEL4:
                        break;

                    case IR_CMD_VOL_UP:
                        volume_up();
                        break;

                    case IR_CMD_VOL_DOWN:
                        volume_down();
                        break;

                    default:
                        break;
                }
            }
        }

        volume_apply_change();
    }
}
