// copyright 2010 morgan quigley, mquigley@cs.stanford.edu
// bsd license blah blah

#include <stdlib.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <avr/interrupt.h>

// hardware connections (completely arbitrary)
// PB7 =  led (on arduino mega)

ISR(USART0_RX_vect)
{
  volatile uint8_t b = UDR0;
  UDR0 = b;
}

int main(void)
{
  wdt_disable();

  DDRB = 0x80;
 
  UBRR0  = 16; // 115k200
  UCSR0A = _BV(U2X0);
  UCSR0B = _BV(RXEN0)  | _BV(RXCIE0) | _BV(TXEN0);
  UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); // 8n1
  sei();

  while (1)
  {
    _delay_ms(100);
    PORTB ^= 0x80;
  }
}

