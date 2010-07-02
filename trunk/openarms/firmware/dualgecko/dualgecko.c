// copyright 2010 morgan quigley, bsd license blah blah
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

int main(void)
{
  OSC.CTRL |= OSC_RC32MEN_bm;
  while (!(OSC.STATUS & OSC_RC32MRDY_bm)) { } // spin until we settle
  CCP = CCP_IOREG_gc;
  CLK.CTRL = 0x01;
  PORTF.DIRSET = PIN3_bm | PIN4_bm | PIN5_bm | PIN7_bm; // rs-485 and led
  PORTF.OUTSET = PIN3_bm | PIN7_bm;

  int bsel = 64; // 1 megabit @ 32mhz clock
  uint8_t bscale = 10;

  USARTF0.BAUDCTRLA = (uint8_t)bsel;
  USARTF0.BAUDCTRLB = (bscale << 4) | (bsel >> 8);
  USARTF0.CTRLA = 0;
  USARTF0.CTRLB = USART_TXEN_bm;
  USARTF0.CTRLC = USART_CHSIZE_8BIT_gc;

  while(1)
  {
    _delay_ms(100);
    PORTF.OUTCLR = PIN7_bm;
    _delay_ms(100);
    PORTF.OUTSET = PIN7_bm;

    PORTF.OUTSET = PIN4_bm; // assert bus
    _delay_ms(1);
    USARTF0.DATA = 'h';
    _delay_ms(10);
    PORTF.OUTCLR = PIN4_bm; // release bus
  }
  return 0; // or not
}  
