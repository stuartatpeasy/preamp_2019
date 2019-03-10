/*
    nvmem.c - abstractions for reading and writing to non-volatile memory (in this case, EEPROM).

    Stuart Wallace <stuartw@atom.net>, March 2019.
*/

#include "nvmem.h"
#include "util/irq.h"
#include <avr/eeprom.h>


// Macro which busy-waits until the EEPROM controller is ready.  Note that this is needed because
// the AVR-libc macro eeprom_busy_wait() appears to be broken for this platform.
#define EEPROM_WAIT_READY()     do { while(NVMCTRL_STATUS & NVMCTRL_EEBUSY_bm) ; } while(0)


// nvmem_update_byte() - wrapper around eeprom_update_byte().  Updates the contents of the EEPROM
// location indicated by <*p> to the value <val>.  Does so with interrupts disabled.
//
void nvmem_update_byte(uint8_t *p, uint8_t val)
{
    interrupt_enable_decrement();
    EEPROM_WAIT_READY();
    eeprom_update_byte(p, val);
    EEPROM_WAIT_READY();
    interrupt_enable_increment();
}


// nvmem_read_byte() - read and return a byte from the EEPROM location indicated by <*p> in an
// interrupt-safe way.
//
uint8_t nvmem_read_byte(uint8_t *p)
{
    uint8_t ret;

    interrupt_enable_decrement();
    ret = eeprom_read_byte(p);
    interrupt_enable_increment();

    return ret;
}
