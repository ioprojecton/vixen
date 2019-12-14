// PURPOSE: VixenLights 3.8 processing with Pullstar8 I2C Servo Motor Board on Arduino UNO
//          I2C and UART modified functions derived from Peter Fleury`s library.
// LICENSE: Apache-2.0 (https://www.apache.org/licenses/LICENSE-2.0.txt)
// 
//     URL: https://github.com/ioprojecton/vixen
//
// HISTORY:
// Surepic - Original version(12/14/2019)
// 
// 
#include <inttypes.h>
#include <compat/twi.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define UART_BAUD_RATE 115200UL

#define UART_BUFFER_OVERFLOW  0x0200

#define UART_NO_DATA          0x0100

#define IODIR 0x00

#define IOCON 0x05

#define mcp_address 0b01000000

#define SCL_CLOCK  400000UL

#define OLAT 0x0A

#define DELIM 0x3C

#define UART_RX_BUFFER_MASK ( UART_RX_BUFFER_SIZE - 1)

#define UART_RX_BUFFER_SIZE 16

#define SET_BIT(_byte,_position) (_byte |= 1 << 7 - _position)

#define ZERO_OUT(_array) memset(_array,0,sizeof(_array))

unsigned int uart_getc(void);

void i2c_write( unsigned char data );

void i2c_stop(void);

void i2c_start(unsigned char address);

unsigned int input_buffer[8];

static volatile unsigned char UART_RxBuf[UART_RX_BUFFER_SIZE];

static volatile unsigned char UART_RxHead;

static volatile unsigned char UART_RxTail;

static volatile unsigned char UART_LastRxError;

unsigned char v;

void setup() {
  // put your setup code here, to run once:
  TWSR = 0;

  TWBR = ((F_CPU / SCL_CLOCK) - 16) / 2;

  UART_RxHead = 0;

  UART_RxTail = 0;

  UCSR0A = _BV(U2X0);

  UBRR0H = 0;

  UBRR0L = 16;

  UCSR0B = _BV(RXCIE0) | _BV(RXEN0);

  UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);

  sei();

  i2c_start(mcp_address);

  i2c_write(IODIR);

  i2c_write(0x00);

  i2c_stop();

  i2c_start(mcp_address);

  i2c_write(IOCON);

  i2c_write(0x020);

  i2c_stop();
}

void loop() {

  ZERO_OUT(input_buffer);

  v = 0;

  while ((unsigned char)uart_getc() != DELIM);

  for (unsigned char i = 0; i < 8; i++) {

    if ((input_buffer[i] = uart_getc()) & UART_NO_DATA) {
      i--;
      continue;
    }

    if (input_buffer[i]) SET_BIT(v, i);
  }

  i2c_start(mcp_address);

  i2c_write(OLAT);

  i2c_write(v);

  i2c_stop();
}

ISR (USART_RX_vect)
{
  unsigned char tmphead;

  unsigned char lastRxError;

  lastRxError = UCSR0A & (_BV(FE0) | _BV(DOR0) | _BV(UPE0) );

  tmphead = (UART_RxHead + 1) & UART_RX_BUFFER_MASK;

  UART_RxHead = tmphead;

  UART_RxBuf[tmphead] = UDR0;

  UART_LastRxError |= lastRxError;
}

unsigned int uart_getc(void)
{
  unsigned char tmptail;
  unsigned char lastRxError;

  if ( UART_RxHead == UART_RxTail ) return UART_NO_DATA;

  tmptail = (UART_RxTail + 1) & UART_RX_BUFFER_MASK;

  lastRxError = UART_LastRxError;

  UART_RxTail = tmptail;

  UART_LastRxError = 0;

  return (lastRxError << 8) + UART_RxBuf[tmptail];
}

void i2c_start(unsigned char address)
{

  TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);

  while (!(TWCR & _BV(TWINT)));

  TWDR = address;

  TWCR = _BV(TWINT) | _BV(TWEN);

  while (!(TWCR & _BV(TWINT)));
}

void i2c_stop(void)
{
  TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTO);

  while (TWCR & _BV(TWSTO));
}

void i2c_write( unsigned char data )
{
  TWDR = data;

  TWCR = _BV(TWINT) | _BV(TWEN);

  while (!(TWCR & _BV(TWINT)));
}
