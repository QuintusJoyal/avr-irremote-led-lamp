// Reference: http://www.rjhcoding.com/avrc-uart.php

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#ifndef BAUDRATE
#define BAUDRATE 9600
#endif

#define BAUD_PRESCALLER (((F_CPU / (BAUDRATE * 16UL))) - 1)

#include <avr/interrupt.h>
#include <avr/io.h>

void uart_init();
void uart_putc(unsigned char data);
void uart_puts(const char* s);
void uart_putU8(uint8_t val);
void uart_putU16(uint16_t val);
void uart_puthex8(uint8_t val);
void uart_puthex16(uint16_t val);

void uart_init()
{
  UBRR0H = (uint8_t)(BAUD_PRESCALLER >> 8);
  UBRR0L = (uint8_t)(BAUD_PRESCALLER);

  UCSR0B = (1 << RXEN0) | (1 << TXEN0);
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void uart_putc(unsigned char data)
{
  // wait for transmit buffer to be empty
  while (!(UCSR0A & (1 << UDRE0)))
    ;

  // load data into transmit register
  UDR0 = data;
}

void uart_puts(const char *s)
{
  // transmit character until NULL is reached
  while (*s > 0)
    uart_putc(*s++);
}

void uart_putU8(uint8_t val)
{
  uint8_t dig1 = '0', dig2 = '0';

  // count value in 100s place
  while (val >= 100)
  {
    val -= 100;
    dig1++;
  }

  // count value in 10s place
  while (val >= 10)
  {
    val -= 10;
    dig2++;
  }

  // print first digit (or ignore leading zeros)
  if (dig1 != '0')
    uart_putc(dig1);

  // print second digit (or ignore leading zeros)
  if ((dig1 != '0') || (dig2 != '0'))
    uart_putc(dig2);

  // print final digit
  uart_putc(val + '0');
}

void uart_putU16(uint16_t val)
{
  uint8_t dig1 = '0', dig2 = '0', dig3 = '0', dig4 = '0';

  // count value in 10000s place
  while (val >= 10000)
  {
    val -= 10000;
    dig1++;
  }

  // count value in 1000s place
  while (val >= 1000)
  {
    val -= 1000;
    dig2++;
  }

  // count value in 100s place
  while (val >= 100)
  {
    val -= 100;
    dig3++;
  }

  // count value in 10s place
  while (val >= 10)
  {
    val -= 10;
    dig4++;
  }

  // was previous value printed?
  uint8_t prevPrinted = 0;

  // print first digit (or ignore leading zeros)
  if (dig1 != '0')
  {
    uart_putc(dig1);
    prevPrinted = 1;
  }

  // print second digit (or ignore leading zeros)
  if (prevPrinted || (dig2 != '0'))
  {
    uart_putc(dig2);
    prevPrinted = 1;
  }

  // print third digit (or ignore leading zeros)
  if (prevPrinted || (dig3 != '0'))
  {
    uart_putc(dig3);
    prevPrinted = 1;
  }

  // print third digit (or ignore leading zeros)
  if (prevPrinted || (dig4 != '0'))
  {
    uart_putc(dig4);
    prevPrinted = 1;
  }

  // print final digit
  uart_putc(val + '0');
}

void uart_puthex8(uint8_t val)
{
  // extract upper and lower nibbles from input value
  uint8_t upperNibble = (val & 0xF0) >> 4;
  uint8_t lowerNibble = val & 0x0F;

  // convert nibble to its ASCII hex equivalent
  upperNibble += upperNibble > 9 ? 'A' - 10 : '0';
  lowerNibble += lowerNibble > 9 ? 'A' - 10 : '0';

  // print the characters
  uart_putc(upperNibble);
  uart_putc(lowerNibble);
}

void uart_puthex16(uint16_t val)
{
  // transmit upper 8 bits
  uart_puthex8((uint8_t)(val >> 8));

  // transmit lower 8 bits
  uart_puthex8((uint8_t)(val & 0x00FF));
}