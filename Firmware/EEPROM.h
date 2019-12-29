#ifndef __EEPROM_H_
#define __EEPROM_H_

#include <avr/eeprom.h>
#include <stdint.h>

class EEPROM
{
public:
    static void init()
    {
        // configure for erase & write in a single atomic operation
        EECR &= ~((1 << EEPM0) | (1 << EEPM1));
    }

    static void writeByte(uint8_t addr, uint8_t data)
    {
        while ((EECR >> EEPE) & 1)
            ; // Wait for completion of previous write.

        EEAR = addr;
        EEDR = data;

        EECR |= (1 << EEMPE); // EEPROM Master Write Enable
        EECR |= (1 << EEPE);  // Start EEPROM write by setting EEWE
    }

    static uint8_t readByte(uint8_t addr)
    {
        while ((EECR >> EEPE) & 1)
            ; //Wait for completion of previous write if any.

        EEAR = addr; //Load the address from where the data needs to be read.
        while ((EECR >> EERE) & 1)
            ; // start EEPROM read by setting EERE

        return EEDR; // Return data from data register
    }

    static void writeBlock(uint8_t addr, uint8_t *data, uint8_t size)
    {
        while (size--)
        {
            writeByte(addr++, *(data++));
        }
    }

    static void readBlock(uint8_t addr, uint8_t *buffer, uint8_t size)
    {
        while (size--)
        {
            *(buffer++) = readByte(addr++);
        }
    }
};
#endif //__EEPROM_H_