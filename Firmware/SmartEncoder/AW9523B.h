#include <stdint.h>
#include <Arduino.h>
#include <Wire.h>

// Used to specify that pins should be left in initial state after power up
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
  AW9523B(uint8_t address, uint8_t resetPin, uint8_t intPin = 0, uint8_t initialState = 0xFF, uint8_t imax = IMAX_19MA) : addr(address),
                                                                                                                  rstn(resetPin),
                                                                                                                  intn(intPin)
  {
    // Get the chip out of reset mode
    if (this->rstn != 0)
    {
      ::pinMode(this->rstn, OUTPUT);
      ::digitalWrite(this->rstn, HIGH);
    }

    if (this->intn != 0)
    {
      ::pinMode(this->intn, INPUT_PULLUP);
    }

    Wire.begin();

    // set the global config to push-pull output
    // mode and the chosen I_MAX setting
    uint8_t gcrVal = (1 << GPOMD) | imax;
    this->writeRegister(GCR, gcrVal);

    if (initialState != UNCHANGED)
    {
      // a 1 in the config register indicates an input,
      // while a 0 indicates an output.
      uint8_t portState = initialState == INPUT ? 0xFF : 0x00;
      this->writeRegister(CONFIG_PORT0, portState);
      this->writeRegister(CONFIG_PORT1, portState);

      // if configured for LED mode, set the
      // register to enable led current drive
      uint8_t ledModeState = initialState == LED_MODE ? 0xFF : 0x00;
      this->writeRegister(LED_MODE_PORT0, ledModeState);
      this->writeRegister(LED_MODE_PORT1, ledModeState);
    }
  }

  void pinMode(uint8_t pin, uint8_t mode)
  {
    uint8_t reg = this->getRegisterAddr(CONFIG_PORT0, pin);
    uint8_t bit = this->getRegisterBit(pin);

    uint8_t val = mode == INPUT ? 1 : 0;
    this->setRegisterBit(reg, bit, val);

    uint8_t ledModeVal = mode == LED_MODE ? 1 : 0;
    uint8_t ledModeRegister = this->getRegisterAddr(LED_MODE_PORT0, pin);
    this->setRegisterBit(ledModeRegister, bit, ledModeVal);
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

  uint8_t readRegister(uint8_t reg)
  {
    Wire.beginTransmission(this->addr);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(this->addr, (uint8_t)1);
    while (Wire.available() == 0)
      ;
    return Wire.read();
  }

  void writeRegister(uint8_t reg, uint8_t val)
  {
    Wire.beginTransmission(this->addr);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
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
