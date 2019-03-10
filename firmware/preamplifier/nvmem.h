#ifndef NVMEM_H_INC
#define NVMEM_H_INC
/*
    nvmem.h - abstractions for reading and writing to non-volatile memory (in this case, EEPROM).

    Stuart Wallace <stuartw@atom.net>, March 2019.
*/

#include <stdint.h>


void nvmem_update_byte(uint8_t *p, uint8_t val);
uint8_t nvmem_read_byte(uint8_t *p);

#endif
