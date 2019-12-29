#ifndef __SERIAL_H__
#define __SERIAL_H__

#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/io.h>

#define RX_BUF_SIZE 50

ISR(USART_RX_vect);

class Serial
{
public:
    static void begin(uint32_t baudRate)
    {
        Serial::rx_read_pos = 0;
        Serial::rx_write_pos = 0;
        uint16_t rate = (((F_CPU / (baudRate * 16UL))) - 1);
        /* Set baud rate */
        UBRRH = (uint8_t)(rate >> 8);
        UBRRL = (uint8_t)rate;
        /* Enable receiver and transmitter */
        UCSRB = (1 << RXEN) | (1 << TXEN) | (1 << RXCIE);
        /* Set frame format: 8data, 1stop bit, no parity */
        UCSRC = (1 << UCSZ1) | (1 << UCSZ0);
    }

    static void putc(unsigned char data)
    {
        /* Wait for empty transmit buffer */
        while (!(UCSRA & (1 << UDRE)))
            ;
        /* Put data into buffer, sends the data */
        UDR = data;
    }

    static uint8_t available()
    {
        return rx_write_pos != rx_read_pos;
    }

    static unsigned char getc()
    {
        // wait until the write buffer is ahead
        // of the read buffer to make sure data is available
        while (rx_read_pos == rx_write_pos)
            ;
        unsigned char data = rx_buf[rx_read_pos++];
        rx_read_pos %= RX_BUF_SIZE;
        return data;
    }

    static void write(unsigned char *data)
    {
        while (*data != '\0')
        {
            putc(*data);
            data++;
        }
    }

    static void write(uint8_t data)
    {
        Serial::write(&data);
    }

    friend void USART_RX_vect(void);

private:
    static unsigned char rx_buf[RX_BUF_SIZE];
    static uint8_t rx_read_pos, rx_write_pos;

    Serial() {}
};

uint8_t Serial::rx_read_pos;
uint8_t Serial::rx_write_pos;
unsigned char Serial::rx_buf[RX_BUF_SIZE];

ISR(USART_RX_vect)
{
    Serial::rx_buf[Serial::rx_write_pos++] = UDR;
    Serial::rx_write_pos %= RX_BUF_SIZE;
}
#endif