// copyright 2010 morgan quigley, bsd license blah blah
#define F_CPU 32000000
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

// these guys will be on the same bus as the dynamixels, so we'll use
// their protocol here as well.
#define MY_ID                12

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

#define MIN_TIMER_TOP        100

volatile uint8_t g_rx_state = RX_STATE_PREAMBLE_1;
volatile uint8_t g_rx_id = 0;
volatile uint8_t g_rx_length = 0;
volatile uint8_t g_rx_instruction = 0;
volatile uint8_t g_rx_checksum = 0;
volatile uint8_t g_rx_param_writepos;
volatile uint8_t g_rx_param[RX_MAX_PARAM];
volatile uint8_t g_tx_pkt[TX_MAX_LENGTH];
volatile uint8_t g_tx_pkt_len = 0;
volatile uint8_t g_tx_pkt_readpos = 0;
volatile uint8_t g_last_byte = 0;

volatile uint8_t g_accel_data[2][8];
volatile uint8_t g_accel_write_slot = 0;

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
  PORTF.OUTSET = PIN4_bm | PIN5_bm; // assert bus and disable receiver
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
      else
      {
        static uint8_t unexpected = 0;
        unexpected++;
      }
      break;
    case RX_STATE_PREAMBLE_2:
      if (b == 0xff)
        g_rx_state = RX_STATE_ID;
      else
      {
        static uint8_t unexpected2 = 0;
        unexpected2++;
        g_rx_state = RX_STATE_PREAMBLE_1; // reset and try to re-sync
      }
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
        g_rx_param_writepos = 0;
        if (g_rx_length > 2)
          g_rx_state = RX_STATE_PARAMETER;
        else
          g_rx_state = RX_STATE_CHECKSUM;
      }
      else
        g_rx_state = RX_STATE_PREAMBLE_1;
      break;
    case RX_STATE_PARAMETER:
      g_rx_checksum += b;
      if (g_rx_param_writepos < RX_MAX_PARAM)
        g_rx_param[g_rx_param_writepos++] = b;
      if (g_rx_param_writepos >= g_rx_length - 2)
        g_rx_state = RX_STATE_CHECKSUM;
      break;
    case RX_STATE_CHECKSUM:
      g_rx_state = RX_STATE_PREAMBLE_1;
      g_rx_checksum = ~g_rx_checksum;
      if (b == g_rx_checksum)
      {
        if (g_rx_id == MY_ID || g_rx_id == 0xfe) // hey! we've got mail!
        {
          if (g_rx_instruction == 0x01)
            send_packet(0); // respond to ping
          else if (g_rx_instruction == 0x02)
          {
            // read data
            uint8_t read_addr = g_rx_param[0];
            uint8_t read_len = g_rx_param[1];
            if (read_addr == 0x25 && read_len == 2) // return encoder value
            {
              uint16_t encoder_latch;
              cli();
              encoder_latch = PORTD.IN ; //TCC0.CNT;
              sei();
              g_tx_pkt[TX_DATA_START  ] = *((uint8_t *)(&encoder_latch)  );
              g_tx_pkt[TX_DATA_START+1] = *((uint8_t *)(&encoder_latch)+1);
              send_packet(2);
            }
            else if (read_addr == 0x2b && read_len == 8) // return accelerometer data
            {
              uint8_t i;
              volatile uint8_t read_slot = g_accel_write_slot ? 0 : 1;
              for (i = 0; i < 8; i++)
                g_tx_pkt[TX_DATA_START+i] = g_accel_data[read_slot][i];
              send_packet(8);
            }
          }
          else if (g_rx_instruction == 0x03)
          {
            if (g_rx_length == 4) // 8-bit write
            {
              const uint8_t val = g_rx_param[1];
              if (g_rx_param[0] == 0x19) // LED on/off
              {
                if (val)
                  PORTF.OUTSET = PIN7_bm;
                else
                  PORTF.OUTCLR = PIN7_bm;
              }
            }
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
  g_last_byte = USARTF0.DATA;
  process_byte(g_last_byte);
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
    PORTF.OUTCLR = PIN4_bm | PIN5_bm; // release bus and re-enable receiver
  }
}

void uart_init()
{
  int bsel = 64; // 1 megabit @ 32mhz clock
  uint8_t bscale = 10;

  PORTF.DIRSET = PIN3_bm | PIN4_bm | PIN5_bm | PIN7_bm; // rs-485 and led
  PORTF.OUTSET = PIN3_bm | PIN7_bm;
  PORTF.OUTCLR = PIN4_bm; // make sure bus is released

  USARTF0.BAUDCTRLA = (uint8_t)bsel;
  USARTF0.BAUDCTRLB = (bscale << 4) | (bsel >> 8);
  USARTF0.CTRLA = USART_RXCINTLVL_LO_gc; // rx fires a low-level interrupt
  USARTF0.CTRLB = USART_TXEN_bm | USART_RXEN_bm;
  USARTF0.CTRLC = USART_CHSIZE_8BIT_gc;
}

void poll_accel()
{
  uint8_t reg_offset = 0;
  PORTD.OUTCLR = PIN4_bm;
  _delay_us(1);
  SPID.DATA = 0x81; // start reading register 1
  while (!(SPID.STATUS & SPI_IF_bm)) { }
  SPID.DATA = 0x00; // dummy
  for (reg_offset = 0; reg_offset < 8; reg_offset++)
  {
    while (!(SPID.STATUS & SPI_IF_bm)) { }
    g_accel_data[g_accel_write_slot][reg_offset] = SPID.DATA;
    if (reg_offset < 7)
      SPID.DATA = 0x00; // dummy
  }
  g_accel_write_slot = g_accel_write_slot ? 0 : 1;
  _delay_us(1);
  PORTD.OUTSET = PIN4_bm;
}

int main(void)
{
  OSC.CTRL |= OSC_RC32MEN_bm;
  while (!(OSC.STATUS & OSC_RC32MRDY_bm)) { } // spin until we settle
  CCP = CCP_IOREG_gc;
  CLK.CTRL = 0x01;

  PORTC.DIRSET = PIN0_bm | PIN1_bm | PIN2_bm;
  PORTE.DIRSET = PIN0_bm | PIN1_bm | PIN2_bm;

  uart_init();

  PMIC.CTRL |= PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm; // enable low-level interrupts

  PORTD.DIRSET = PIN4_bm | PIN5_bm | PIN7_bm;
  PORTD.OUTSET = PIN4_bm | PIN7_bm;
  SPID.CTRL = SPI_ENABLE_bm | SPI_MODE0_bm | SPI_MODE1_bm | SPI_MASTER_bm | SPI_PRESCALER_DIV64_gc;

  PORTD.DIRCLR = PIN0_bm | PIN1_bm | PIN2_bm;

  PORTD.PIN0CTRL = PORT_ISC_BOTHEDGES_gc; // quadrature index
  PORTD.PIN1CTRL = PORT_ISC_LEVEL_gc; // quadrature A
  PORTD.PIN2CTRL = PORT_ISC_LEVEL_gc; // quadrature B

  EVSYS.CH0MUX = EVSYS_CHMUX_PORTD_PIN1_gc; // A & B inputs to quadrature decoder
  EVSYS.CH1MUX = EVSYS_CHMUX_PORTD_PIN0_gc; // index input to quadrature decoder
  EVSYS.CH0CTRL = EVSYS_QDEN_bm | EVSYS_QDIEN_bm | EVSYS_DIGFILT_2SAMPLES_gc;
  EVSYS.CH1CTRL = EVSYS_DIGFILT_2SAMPLES_gc;

  TCC0.CTRLD = TC_EVACT_QDEC_gc;
  TCC0.PER = 1024 * 4 - 1;
  TCC0.CTRLA = TC_CLKSEL_DIV1_gc;

  sei();

  while(1)
  {
    /*
    _delay_ms(10);
    poll_accel();
    */
    _delay_ms(200);
    PORTF.OUTTGL = PIN7_bm;
  }
  return 0; // or not
}  

