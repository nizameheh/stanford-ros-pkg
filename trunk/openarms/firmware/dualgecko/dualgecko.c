// copyright 2010 morgan quigley, bsd license blah blah
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

// these guys will be on the same bus as the dynamixels, so we'll use
// their protocol here as well.
#define MY_ID                10

#define RX_STATE_PREAMBLE_1  1
#define RX_STATE_PREAMBLE_2  2
#define RX_STATE_ID          3
#define RX_STATE_LENGTH      4
#define RX_STATE_INSTRUCTION 5
#define RX_STATE_PARAMETER   6
#define RX_STATE_CHECKSUM    7
#define RX_MAX_PARAM         50

#define TX_MAX_LENGTH        50
#define TX_DATA_START        5
#define TX_EXTRA_DELAY_US    10

volatile uint8_t g_rx_state = RX_STATE_PREAMBLE_1;
volatile uint8_t g_rx_id = 0, g_rx_length = 0;
volatile uint8_t g_rx_instruction = 0, g_rx_checksum = 0;
volatile uint8_t g_rx_param_writepos, g_rx_param[RX_MAX_PARAM];
volatile uint8_t g_tx_pkt[TX_MAX_LENGTH], g_tx_pkt_len = 0;
volatile uint8_t g_tx_pkt_readpos = 0;

// this assumes that g_tx_pkt and g_tx_pkt_len have been stuffed
void send_packet(uint8_t tx_data_len)
{
  uint8_t i, checksum = 0;
  g_tx_pkt_len = TX_DATA_START + tx_data_len + 1;
  g_tx_pkt[0] = 0xff;
  g_tx_pkt[1] = 0xff;
  g_tx_pkt[2] = MY_ID;
  g_tx_pkt[3] = 2 + tx_data_len;
  g_tx_pkt[4] = 0; // no errors
  for (i = 2; i < TX_DATA_START + tx_data_len; i++)
    checksum += g_tx_pkt[i];
  g_tx_pkt[g_tx_pkt_len-1] = ~checksum;
  _delay_us(TX_EXTRA_DELAY_US);
  PORTF.OUTSET = PIN4_bm; // assert bus
  _delay_us(1); // wait another bit length for bus to settle (?)
  g_tx_pkt_readpos = 1;
  USARTF0.CTRLA = USART_TXCINTLVL_LO_gc; // turn on TXC, turn off RXC
  USARTF0.DATA = g_tx_pkt[0];
}

void process_byte(uint8_t b)
{
  switch(g_rx_state)
  {
    case RX_STATE_PREAMBLE_1:
      if (b == 0xff)
        g_rx_state = RX_STATE_PREAMBLE_2;
      break;
    case RX_STATE_PREAMBLE_2:
      if (b == 0xff)
        g_rx_state = RX_STATE_ID;
      else
        g_rx_state = RX_STATE_PREAMBLE_1; // reset and try to re-sync
      break;
    case RX_STATE_ID:
      if (b < 254)
      {
        g_rx_id = g_rx_checksum = b;
        g_rx_state = RX_STATE_LENGTH;
      }
      else
        g_rx_state = RX_STATE_PREAMBLE_1;
      break;
    case RX_STATE_LENGTH:
      if (b < 100)
      {
        g_rx_length = b;
        g_rx_checksum += b;
        g_rx_state = RX_STATE_INSTRUCTION;
      }
      else
        g_rx_state = RX_STATE_PREAMBLE_1;
      break;
    case RX_STATE_INSTRUCTION:
      if (b < 0x07 || b == 0x83)
      {
        g_rx_instruction = b;
        g_rx_checksum += b;
        g_rx_state = RX_STATE_PARAMETER;
      }
      else
        g_rx_state = RX_STATE_PREAMBLE_1;
      break;
    case RX_STATE_PARAMETER:
      g_rx_checksum += b;
      if (g_rx_param_writepos < RX_MAX_PARAM)
        g_rx_param[g_rx_param_writepos++] = b;
      if (g_rx_param_writepos >= g_rx_length + 2)
        g_rx_state = RX_STATE_CHECKSUM;
      break;
    case RX_STATE_CHECKSUM:
      g_rx_state = RX_STATE_PREAMBLE_1;
      if (b == ~g_rx_checksum)
      {
        if (g_rx_id == MY_ID)
        {
          // successful packet receipt
          PORTF.OUTTGL = PIN7_bm;
          // do something productive here
          if (g_rx_instruction == 0x01)
            send_packet(0); // respond to ping
          else if (g_rx_instruction == 0x02)
          {
            // read data
          }
          else if (g_rx_instruction == 0x03)
          {
            // write data
          }
          else if (g_rx_instruction == 0x06)
          {
            // reset
          }
          else if (g_rx_instruction == 0x83)
          {
            // sync write
          }
        }
      }
      break;
    default:
      // shouldn't ever get here
      g_rx_state = RX_STATE_PREAMBLE_1;
      break;
  }
}

ISR(USARTF0_RXC_vect)
{
  process_byte(USARTF0.DATA);
}

ISR(USARTF0_TXC_vect)
{
  if (g_tx_pkt_readpos < g_tx_pkt_len)
    USARTF0.DATA = g_tx_pkt[g_tx_pkt_readpos++]; // keep blasting
  else
  {
    USARTF0.CTRLA = USART_RXCINTLVL_LO_gc; // turn off the TXC interrupt
    // de-assert the bus after we hold it for one extra bit
    _delay_us(1);
    PORTF.OUTCLR = PIN4_bm; // release bus
  }
}

int main(void)
{
  OSC.CTRL |= OSC_RC32MEN_bm;
  while (!(OSC.STATUS & OSC_RC32MRDY_bm)) { } // spin until we settle
  CCP = CCP_IOREG_gc;
  CLK.CTRL = 0x01;
  PORTF.DIRSET = PIN3_bm | PIN4_bm | PIN5_bm | PIN7_bm; // rs-485 and led
  PORTF.OUTSET = PIN3_bm | PIN7_bm;
  PORTF.OUTCLR = PIN4_bm; // make sure bus is released

  int bsel = 64; // 1 megabit @ 32mhz clock
  uint8_t bscale = 10;

  USARTF0.BAUDCTRLA = (uint8_t)bsel;
  USARTF0.BAUDCTRLB = (bscale << 4) | (bsel >> 8);
  USARTF0.CTRLA = USART_RXCINTLVL_LO_gc; // rx fires a low-level interrupt
  USARTF0.CTRLB = USART_TXEN_bm;
  USARTF0.CTRLC = USART_CHSIZE_8BIT_gc;

  PMIC.CTRL |= PMIC_LOLVLEN_bm; // enable low-level interrupts
  sei();

  while(1)
  {
/*
    PORTF.OUTSET = PIN4_bm; // assert bus
    _delay_ms(1);
    USARTF0.DATA = 'h';
    _delay_ms(10);
    PORTF.OUTCLR = PIN4_bm; // release bus
*/
  }
  return 0; // or not
}  
