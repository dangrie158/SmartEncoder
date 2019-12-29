#ifndef __AW9523B_H__
#define __AW9523B_H__

#include <stdint.h>
#include <util/delay.h>
#include "Wire.h"

// Used to specify that pins should be left in initial state after power up
#define INPUT 0
#define OUTPUT 1
#define UNCHANGED 0x55
#define LED_MODE 0x56

#define INPUT_PORT0 0x00
#define INPUT_PORT1 0x01
#define OUTPUT_PORT0 0x02
#define OUTPUT_PORT1 0x03
#define CONFIG_PORT0 0x04
#define CONFIG_PORT1 0x05
#define INT_PORT0 0x06
#define INT_PORT1 0x07

// global control register
#define GCR 0x11

#define LED_MODE_PORT0 0x12
#define LED_MODE_PORT1 0x13

// LED dimmer current setting for pin 0,
// other pins are subsequent to this register
// so the corresponding register for pin X is
// DIM0 + X
#define DIM0 0x20

// current limit in LED Mode. I_MAX is ~37mA,
// other steps are 3/4, 1/2 and 1/4 of that value
#define IMAX_37MA 0b00
#define IMAX_28MA 0b01
#define IMAX_19MA 0b10
#define IMAX_9MA 0b11

#define GPOMD 4

class AW9523B
{
public:
  AW9523B(uint8_t address, uint8_t resetPin, volatile uint8_t *rstDDR = 0, volatile uint8_t *rstPORT = 0, uint8_t intPin = 0, uint8_t imax = IMAX_19MA) : addr(address),
                                                                                                                                                          rstn(resetPin),
                                                                                                                                                          intn(intPin),
                                                                                                                                                          rstPORT(rstPORT)
  {
    // Get the chip out of reset mode
    if (this->rstn != 0)
    {
      *rstDDR |= (1 << rstn);
      *rstPORT |= (1 << rstn);
    }

    // if (this->intn != 0)
    // {
    //   ::pinMode(this->intn, INPUT_PULLUP);
    // }

    Wire::init();

    // set the global config to push-pull output
    // mode and the chosen I_MAX setting
    uint8_t gcrVal = (1 << GPOMD) | imax;
    this->writeRegister(GCR, gcrVal);
  }

  void reset()
  {
    *rstPORT &= ~(1 << this->rstn);
    _delay_ms(1);
    *rstPORT |= (1 << this->rstn);
  }

  void pinMode(uint8_t pin, uint8_t mode)
  {
    uint8_t reg = this->getRegisterAddr(CONFIG_PORT0, pin);
    uint8_t bit = this->getRegisterBit(pin);

    uint8_t val = mode == INPUT ? 1 : 0;
    this->setRegisterBit(reg, bit, val);

    uint8_t ledModeVal = mode == LED_MODE ? 0 : 1;
    uint8_t ledModeRegister = this->getRegisterAddr(LED_MODE_PORT0, pin);
    this->setRegisterBit(ledModeRegister, bit, ledModeVal);
  }

  void portMode(uint8_t port, uint8_t mode)
  {
    uint8_t reg = port == 0 ? CONFIG_PORT0 : CONFIG_PORT1;
    uint8_t ledReg = port == 0 ? LED_MODE_PORT0 : LED_MODE_PORT1;
    uint8_t val = mode == INPUT ? 0xFF : 0x00;
    uint8_t ledModeVal = mode == LED_MODE ? 0x00 : 0xFF;

    this->writeRegister(reg, val);
    this->writeRegister(ledReg, ledModeVal);
  }

  void digitalWrite(uint8_t pin, uint8_t val)
  {
    uint8_t reg = this->getRegisterAddr(OUTPUT_PORT0, pin);
    uint8_t bit = this->getRegisterBit(pin);

    this->setRegisterBit(reg, bit, !!val);
  }

  void analogWrite(uint8_t pin, int val)
  {
    this->writeRegister(DIM0 + pin, val);
  }

  int digitalRead(uint8_t pin)
  {
    uint8_t reg = this->getRegisterAddr(INPUT_PORT0, pin);
    uint8_t bit = this->getRegisterBit(pin);

    uint8_t val = readRegister(reg);
    return (val >> bit) & 1;
  }

private:
  uint8_t addr;
  uint8_t rstn;
  uint8_t intn;
  volatile uint8_t *rstPORT;

  uint8_t readRegister(uint8_t reg)
  {
    Wire::start(this->addr, 0);

    Wire::write(reg);
    Wire::restart(this->addr, (uint8_t)1);
    uint8_t val = Wire::read();
    Wire::stop();
    return val;
  }

  void writeRegister(uint8_t reg, uint8_t val)
  {
    Wire::start(this->addr, 0);
    Wire::write(reg);
    Wire::write(val);
    Wire::stop();
  }

  void setRegisterBit(uint8_t reg, uint8_t bit, uint8_t val)
  {
    uint8_t current = readRegister(reg);
    if (val)
    {
      current |= 1 << bit;
    }
    else
    {
      current &= ~(1 << bit);
    }
    writeRegister(reg, current);
  }

  uint8_t getRegisterAddr(uint8_t reg, uint8_t pin)
  {

    if (pin >= 8)
    {
      reg++;
    }
    return reg;
  }

  uint8_t getRegisterBit(uint8_t pin)
  {
    return pin % 8;
  }
};

#endif //__AW9523B_H__