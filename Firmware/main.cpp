#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include "AW9523B.h"
#include "Encoder.h"
#include "Serial.h"

#define MODE_PIN1 PB4 // PB4 (Display Mode Select)
#define MODE_PIN2 PB3 // PB3 (Output Mode Select)

#define ENC_SW PB0 // PB0
#define ENC_A PD3  // PD3 / INT1
#define ENC_B PD2  // PD2 / INT0

#define OUT_P2 PD1 // PD1 / TX / ENC_A
#define OUT_P3 PD0 // PD0 / RX / ENC_B
#define OUT_P4 PB1 // PB1 / ENC_BTN

#define ENCODER_STEP_DELAY_MS 1

#define NUM_LEDS 16

struct BootConfig
{
    enum
    {
        D_BAR,
        D_DOT
    } display;
    enum
    {
        O_SERIAL,
        O_PASSTHROUGH
    } output;
};

#define SETTINGS_MAGIC 0x55
#define SETTINGS_START 0x00
struct Settings
{
    //used to determine if the EEPROM was ever filled with default values
    uint8_t initialized;
    uint16_t state;
    uint16_t stepSize;
};

AW9523B leds(0x58, PB6, &DDRB, &PORTB);
Encoder encoder(PD3, PD2, &PORTD, &DDRD);
long encoderPos = -999;
BootConfig conf;
Settings settings;

void saveSettings(Settings *defaults)
{
    //EEPROM.put(SETTINGS_START, *defaults);
}

/**
 * load the settings from the EEPROM and make sure 
 * the EEPROM was initialized with the default
 * values at least once
 */
void loadSettings(Settings *defaults)
{
    //EEPROM.get(SETTINGS_START, defaults);
    if (defaults->initialized != SETTINGS_MAGIC)
    {
        defaults->initialized = SETTINGS_MAGIC;
        defaults->state = 0;
        defaults->stepSize = 256;
        saveSettings(defaults);
    }
}

void setup()
{
    // reset the I/O expander set
    // set both port to LED drive mode
    leds.reset();
    leds.portMode(0, LED_MODE);
    leds.portMode(1, LED_MODE);

    // setup pin IO direction
    // Mode Pin 1
    DDRB &= ~(1 << PB4);
    PORTB |= (1 << PB4);

    // Mode Pin 2
    DDRB &= ~(1 << PB3);
    PORTB |= (1 << PB3);

    // Encoder Switch
    DDRB &= ~(1 << PB0);
    PORTB |= (1 << PB0);

    // Encoder Pin A
    DDRD &= ~(1 << PD3);
    PORTD |= (1 << PD3);

    // Encoder Pin B
    DDRD &= ~(1 << PD2);
    PORTD |= (1 << PD2);

    // Output Pin 2
    DDRD |= (1 << PD1);

    // Output Pin 3
    DDRD |= (1 << PD0);

    // Output Pin 4
    DDRB |= (1 << PB1);

    // get the bootup BootConfiguration
    conf.display = (!((PINB >> PB4) & 1)) ? BootConfig::D_BAR : BootConfig::D_DOT;
    conf.output = (!((PINB >> PB3) & 1)) ? BootConfig::O_SERIAL : BootConfig::O_PASSTHROUGH;

    // initialize to default values
    loadSettings(&settings);

    if (conf.output == BootConfig::O_SERIAL)
    {
        Serial::begin(9600);
    }
}

void loop()
{
    static uint8_t encLineState = false;
    static uint16_t lastEncVal = 0;

    int8_t steps = encoder.getValue() - lastEncVal;
    lastEncVal += steps;

    //output the new data on the selected method
    if (conf.output == BootConfig::O_SERIAL)
    {
    }
    else
    {
        //Passthrough mode. simply copy the states to the output port
        if ((PINB >> PB0) & 1)
        {
            PORTB |= (1 << PB1);
        }
        else
        {
            PORTB &= ~(1 << PB1);
        }

        uint8_t abs_steps = abs(steps);
        uint8_t dir = steps > 0 ? true : false;

        // emulate a gray code output from the encoder
        uint8_t leadingEdgeLine = dir ? PD1 : PD0;
        uint8_t trailingEdgeLine = dir ? PD0 : PD1;
        while (abs_steps > 0)
        {
            encLineState = !encLineState;
            if (encLineState)
            {
                PORTD |= (1 << leadingEdgeLine);
                _delay_ms(ENCODER_STEP_DELAY_MS);
                PORTD |= (1 << trailingEdgeLine);
                _delay_ms(ENCODER_STEP_DELAY_MS);
            }
            else
            {
                PORTD &= ~(1 << leadingEdgeLine);
                _delay_ms(ENCODER_STEP_DELAY_MS);
                PORTD &= ~(1 << trailingEdgeLine);
                _delay_ms(ENCODER_STEP_DELAY_MS);
            }
            abs_steps--;
        }
    }

    //update the LED display
    settings.state += settings.stepSize * steps;
    if (conf.display == BootConfig::D_BAR)
    {
        uint8_t fullOnLeds = settings.state / (256 * NUM_LEDS);
        uint8_t lastLEDState = settings.state % 256;

        leds.analogWrite(fullOnLeds, lastLEDState);
        while (fullOnLeds > 0)
        {
            leds.analogWrite(fullOnLeds - 1, 255);
            fullOnLeds--;
        }
    }
    else
    {
        uint8_t fullOnLed = settings.state / (256 * NUM_LEDS);
        leds.analogWrite(fullOnLed, 255);
        if (fullOnLed > 0)
        {
            leds.analogWrite(fullOnLed - 1, 255 - (settings.state % 256));
        }

        if (fullOnLed < NUM_LEDS - 1)
        {
            leds.analogWrite(fullOnLed + 1, settings.state % 256);
        }
    }
}

int main()
{
    setup();
    while (1)
    {
        loop();
    }
}