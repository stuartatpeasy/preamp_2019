/*
    main.c - entry point

    Stuart Wallace <stuartw@atom.net>, September 2018.
*/

#include "platform.h"                       // for MODULE_TYPE
#include "commands.h"
#include "controls.h"
#include "expander.h"
#include "ir.h"
#include "shiftreg.h"
#include "lib/adc.h"
#include "lib/clk.h"
#include "lib/debug.h"
#include "lib/gpio.h"
#include "lib/spi.h"
#include "lib/vref.h"
#include "util/irq.h"
#include <avr/sleep.h>
#include <stdint.h>
#include <util/delay.h>


static void firmware_main(void);
static void power_off();
static void power_off_loop();
static void power_on();
static void power_on_loop();

int8_t g_irq_state = 0;                     // Used in util/irq.h
uint8_t power = 0;                          // System power state: 0=off, !0=on


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


static void firmware_main()
{
    power = 0;              // Start in "off" state

    // Make power button pin an input, and enable pull-up; make regulator-enable signal an output
    // and set it to 0 to disable the regulators.
    gpio_set_pullup(PIN_PB_POWER, 1);
    gpio_clear(PIN_REG_EN);
    gpio_make_input(PIN_PB_POWER);
    gpio_make_output(PIN_REG_EN);

    // Initialise SPI port 0 - this is the control interface for the PGA2311 programmable-gain
    // amplifier, the TPIC6B595 shift-register/driver, and the MCP23S17 port expander.
    spi0_configure_master(PinsetDefault, SPIClkDiv4);
    spi0_port_activate(1);
    spi0_enable(1);

    // TODO: consider moving these init calls into the power_on() function.
    expander_init();
    shiftreg_init();
    controls_init();
    ir_init();

    set_sleep_mode(SLEEP_MODE_IDLE);
    interrupt_enable_increment();

    while(1)
    {
        power ? power_on_loop() : power_off_loop();
        sleep_mode();
    }
}


// power_on_loop() - this function is executed repeatedly while the system is switched on.  It
// handles commands received from the infrared remote control, and monitors physical controls
// (buttons / encoders / potentiometers) in order to update the state of the system.
//
static void power_on_loop()
{
    const Command_t cmd = get_command();
    static uint8_t mute = 0;

    switch(cmd)
    {
        case CmdPower:
            power_off();        // Switch off
            return;

        case CmdMute:
            mute = !mute;
            // TODO: mute/un-mute
            break;

        case CmdChannel1:
            set_channel(1);
            break;

        case CmdChannel2:
            set_channel(2);
            break;

        case CmdChannel3:
            set_channel(3);
            break;

        case CmdChannel4:
            set_channel(4);
            break;

        case CmdVolUp:
            volume_up();
            break;

        case CmdVolDown:
            volume_down();
            break;

        default:
            break;
    }

    volume_apply_change();
}


// power_off_loop() - this function is executed repeatedly while the system is powered off.  It
// waits until it receives a power-on command, either from the infrared receiver or via a button-
// press, and then initiates power-up by calling power_on().
//
static void power_off_loop()
{
    // If we have received an IR command addressed to this device, and it's a "power" command, or
    // the power button has been pressed: switch on.
    if(get_command() == CmdPower)
        power_on();     // Switch on
}


// power_on() - handle a request to switch on the system.
//
static void power_on()
{
    // TODO: handle switch-on
    gpio_set(PIN_REG_EN);               // Switch on regulators
    _delay_ms(500);                     // Wait for regulators to settle

    // - switch off all shift register outputs
    expander_sr_output_enable(1);       // Enable TPIC6B595 shift register outputs
    set_channel_from_nv();

    // Wait for power button to be released, so that the code in power_on_loop() doesn't switch the
    // system off again.
    while(!power_button_released())
        ;

    power = 1;
}


// power_off() - handle a request to switch off the system.
//
static void power_off()
{
    set_channel(0);                     // Disconnect inputs and outputs
    expander_sr_output_enable(0);       // Disable TPIC6B595 shift register outputs

    gpio_clear(PIN_REG_EN);             // Disable regulators

    // Wait for power button to be released, so that the code in power_off_loop() doesn't switch
    // the system off again.
    while(!power_button_released())
        ;

    power = 0;
}
