/*
    ir.c - receives infrared remote control commands using the RC-5 protocol

    Stuart Wallace <stuartw@atom.net>, March 2019.

    The infrared receiver uses a Vishay TSOP58338 receiver/demodulator module which demodulates the
    36kHz carrier and produces a logic-level output.  This output is inverted, meaning that its
    resting/idle state is logic 1.  This module contains code which runs in interrupt context to
    decode the RC-5 data from the demodulator output.
*/

#include "platform.h"
#include "ir.h"
#include "commands.h"
#include "lib/debug.h"
#include "lib/gpio.h"
#include <avr/interrupt.h>


#define IR_ADDR             (0x15)          // RC-5 address for infrared commands

// Extract the address field (bits 6-10) from an RC-5 packet
#define IR_ADDRESS(x)       (((x) >> 6) & 0x1f)

// Extract the command field (bits 0-5) from an RC-5 packet
#define IR_COMMAND(x)       ((x) & 0x3f)

// Evaluate to non-zero if the "toggle" bit (bit 11) is set; zero otherwise
#define IR_TOGGLE(x)        ((x) & 0x800)

static void isr_ir_rx();
static void ir_start_lockout();


// IRRXState_t - enumeration of states in the state machine controlling the IR command-reception
// process.
//
typedef enum IRRXState
{
    IRRXState_Lockout,
    IRRXState_Idle,
    IRRXState_Start1,
    IRRXState_Start2,
    IRRXState_Toggle,
    IRRXState_Addr0,
    IRRXState_Addr1,
    IRRXState_Addr2,
    IRRXState_Addr3,
    IRRXState_Addr4,
    IRRXState_Data0,
    IRRXState_Data1,
    IRRXState_Data2,
    IRRXState_Data3,
    IRRXState_Data4,
    IRRXState_Data5
} IRRXState_t;


// Duration of IR RX lockout, specified as a timer count value.  The lockout duration is 50ms.
// At reasonable clock rates, this would cause timer B's counter to overflow.  The lockout is
// therefore implemented as 50 1ms delays.  During the lockout period, any data received from the
// IR port is ignored.
#define IRRX_LOCKOUT_CCMP_VAL           (F_CPU / 1000)   // Timer compare value for 1ms delay
#define IRRX_LOCKOUT_INITIAL_COUNT      (50)             // Number of 1ms delays in lockout

static volatile IRRXState_t state;
static volatile uint8_t lockout_count;
volatile uint16_t raw_data, rx_data;


// ir_init() - initialise IO and state machine to prepare to receive commands from the infrared
// interface.
//
void ir_init()
{
    state = IRRXState_Idle;
    gpio_make_input(PIN_IR_RX);
    gpio_set_sense(PIN_IR_RX, GPIOSenseFalling);
}


// ir_start_lockout() - begin a "lockout" period, during which activity on the IR RX port will be
// ignored.  This is used to enforce spacing between reception of successive commands, and to
// ignore invalid data sequences.
//
static void ir_start_lockout()
{
    state = IRRXState_Lockout;
    lockout_count = IRRX_LOCKOUT_INITIAL_COUNT;
    TCB0_CCMP = IRRX_LOCKOUT_CCMP_VAL;
}


// isr_ir_rx() - handle the first edge of a received IR command packet.  This function is called
// only at the start of IR data reception.  It disables the port pin change-of-state interrupt
// which caused it to be triggered, and then sets up a compare interrupt on timer B so that the
// data in the IR packet can be read.
//
static void isr_ir_rx()
{
    gpio_set_sense(PIN_IR_RX, GPIOSenseIntDisable);

    // Received a high-to-low transition on the IR RX pin, indicating the start of an IR command.
    // If the IR command-processor state machine is idle, configure the timer to trigger an
    // interrupt after 0.25 RC-5 bit times.  If the IR RX pin is still low at this point, we will
    // start receiving an IR command.  Otherwise we will assume that the received transition was a
    // glitch.
    if(state == IRRXState_Idle)
    {
        // One RC-5 bit time equals 64 cycles of a 36kHz wave, which equals 1778us or one cycle of
        // a 562.5Hz wave.  Timer B will be configured to use the peripheral clock as its clock
        // source.  The compare value for the timer is therefore:
        //
        // 0.25 * (f(pclk) / 562.5) = f(pclk) / 2250.
        TCB0_CCMP = F_CPU / 2250;

        // Ensure that various control flags have their default values
        TCB0_CTRLB = 0;

        // Set peripheral clock as timer B clock source and enable timer B
        TCB0_CTRLA = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm;

        // Enable timer B interrupt
        TCB0_INTCTRL |= TCB_CAPT_bm;

        state = IRRXState_Start1;
    }
}


// ir_get_cmd() - if the last IR command packet returned by the demodulator was addressed to this
// device, return the command code from the packet; otherwise return zero.  In all cases, discard
// the packet.
//
Command_t ir_get_cmd()
{
    const uint16_t ret = rx_data;
    rx_data = 0;

    return (Command_t) ((IR_ADDRESS(ret) == IR_ADDR) ? IR_COMMAND(ret) : 0);
}


// ISR for transitions on port C pins.  If a high-to-low transition is seen on the IR RX pin, call
// isr_ir_rx() to begin receiving a command packet.
//
ISR(PORTC_PORT_vect)            // FIXME - find a way to get vect from pin
{
    const uint8_t flags = PORTC_INTFLAGS;

    PORTC_INTFLAGS = 0xff;      // FIXME - add a fn to gpio.c to do this
    if(flags & gpio_pin_bit(PIN_IR_RX))
        isr_ir_rx();

}


// ISR for timer B.  The timer B interrupt is fired at various intervals during command reception
// and lockout in order to read IR data and impose timing constraints.
//
ISR(TCB0_INT_vect)
{
    TCB0_INTFLAGS |= TCB_CAPT_bm;       // Acknowledge interrupt

    switch(state)
    {
        case IRRXState_Idle:
            // Ignore interrupt
            break;

        case IRRXState_Lockout:
            // Lockout occurs in units of 1ms; the number of units is specified by lockout_count.
            if(!--lockout_count)
            {
                // The end of the lockout period has been reached.  Prepare for reception of new IR
                // data by resetting the IR RX state machine, disabling the timer interrupt and re-
                // enabling the interrupt on falling transitions of the IR RX input pin.
                state = IRRXState_Idle;
                TCB0_INTCTRL &= ~TCB_CAPT_bm;
                gpio_set_sense(PIN_IR_RX, GPIOSenseFalling);
            }
            break;

        case IRRXState_Start1:
            // This interrupt is triggered 0.25 RC-5 bit-times after a transition on the IR RX
            // input pin.  If the IR RX input pin is still asserted (low) at this point then we
            // have received a start bit; otherwise we assume that this was a glitch.
            if(!gpio_read(PIN_IR_RX))
            {
                // Amend the timer compare value so that an interrupt is generated once per RC-5
                // bit time.  This enables us to read subsequent bits.
                TCB0_CCMP = F_CPU / 563;        // The RC-5 bit rate is 562.5Hz
                state = IRRXState_Start2;
            }
            else
                ir_start_lockout();
            break;

        case IRRXState_Start2:
            // This interrupt is triggered at the appropriate time to read the value of the second
            // start bit in the RC-5 stream.  This should be a logic 1, which will read as a zero.
            // If the bit value is not zero, assume a glitch and lock out the receiver.
            if(!gpio_read(PIN_IR_RX))
            {
                state = IRRXState_Toggle;
                raw_data = 0;
            }
            else
                ir_start_lockout();
            break;

        case IRRXState_Toggle:
        case IRRXState_Addr0:
        case IRRXState_Addr1:
        case IRRXState_Addr2:
        case IRRXState_Addr3:
        case IRRXState_Addr4:
        case IRRXState_Data0:
        case IRRXState_Data1:
        case IRRXState_Data2:
        case IRRXState_Data3:
        case IRRXState_Data4:
            raw_data <<= 1;
            raw_data |= gpio_read(PIN_IR_RX) ? 0 : 1;
            ++state;
            break;

        case IRRXState_Data5:
            raw_data <<= 1;
            raw_data |= gpio_read(PIN_IR_RX) ? 0 : 1;
            rx_data = raw_data;
            ir_start_lockout();
            debug_puthex_word(raw_data);
            debug_putchar('\n');
            break;
    }
}
