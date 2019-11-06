#ifndef __ENCODER_H__
#define __ENCODER_H__

#include <stdint.h>
#include <avr/interrupt.h>

#define ENC_ACCEL_TOP 3072
#define ENC_ACCEL_INC 25
#define ENC_ACCEL_DEC 2

class Encoder
{
public:
    Encoder(uint8_t channelA, uint8_t channelB, volatile uint8_t *pinReg, volatile uint8_t *ddrReg)
        : pinA(channelA), pinB(channelB), pinReg(pinReg), steps(2), delta(0), last(0), acceleration(0)
    {
        *ddrReg |= (1 << pinA) | (1 << pinB);

        if ((*pinReg >> pinA) & 1)
        {
            last = 3;
        }

        if ((*pinReg >> pinB) & 1)
        {
            last ^= 1;
        }
    }

    void service()
    {
        bool moved = false;

        acceleration -= ENC_ACCEL_DEC;
        if (acceleration & 0x8000)
        { // handle overflow of MSB is set
            acceleration = 0;
        }

        int8_t curr = 0;

        if ((*pinReg >> pinA) & 1)
        {
            curr = 3;
        }

        if ((*pinReg >> pinB) & 1)
        {
            curr ^= 1;
        }

        int8_t diff = last - curr;

        if (diff & 1)
        { // bit 0 = step
            last = curr;
            delta += (diff & 2) - 1; // bit 1 = direction (+/-)
            moved = true;
        }

        if (moved)
        {
            // increment accelerator if encoder has been moved
            if (acceleration <= (ENC_ACCEL_TOP - ENC_ACCEL_INC))
            {
                acceleration += ENC_ACCEL_INC;
            }
        }
    }

    int16_t getValue(void)
    {
        int16_t val;

        cli();
        val = delta;

        if (steps == 2)
            delta = val & 1;
        else if (steps == 4)
            delta = val & 3;
        else
            delta = 0; // default to 1 step per notch

        sei();

        if (steps == 4)
            val >>= 2;
        if (steps == 2)
            val >>= 1;

        int16_t r = 0;
        int16_t accel = acceleration >> 8;

        if (val < 0)
        {
            r -= 1 + accel;
        }
        else if (val > 0)
        {
            r += 1 + accel;
        }

        return r;
    }

private:
    uint8_t pinA, pinB;
    volatile uint8_t *pinReg;

    uint8_t steps;
    volatile int16_t delta;
    volatile int16_t last;
    volatile uint16_t acceleration;
};

#endif