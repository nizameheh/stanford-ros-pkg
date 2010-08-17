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

#define MIN_TIMER_TOP        50

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

volatile uint16_t g_tgt_vel[2];
volatile uint16_t g_cur_vel[2];
volatile uint8_t g_motor_enabled[2];
volatile uint32_t g_motor_pos[2];
volatile uint8_t g_motor_dir[2];

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
          PORTF.OUTTGL = PIN7_bm;
          if (g_rx_instruction == 0x01)
            send_packet(0); // respond to ping
          else if (g_rx_instruction == 0x02)
          {
            // read data
            uint8_t read_addr = g_rx_param[0];
            uint8_t read_len = g_rx_param[1];
            if (read_addr == 0x24 && read_len == 8) // return motor positions
            {
              g_tx_pkt[TX_DATA_START  ] = *((uint8_t *)(&g_motor_pos[0])  );
              g_tx_pkt[TX_DATA_START+1] = *((uint8_t *)(&g_motor_pos[0])+1);
              g_tx_pkt[TX_DATA_START+2] = *((uint8_t *)(&g_motor_pos[0])+2);
              g_tx_pkt[TX_DATA_START+3] = *((uint8_t *)(&g_motor_pos[0])+3);
              g_tx_pkt[TX_DATA_START+4] = *((uint8_t *)(&g_motor_pos[1])  );
              g_tx_pkt[TX_DATA_START+5] = *((uint8_t *)(&g_motor_pos[1])+1);
              g_tx_pkt[TX_DATA_START+6] = *((uint8_t *)(&g_motor_pos[1])+2);
              g_tx_pkt[TX_DATA_START+7] = *((uint8_t *)(&g_motor_pos[1])+3);
              send_packet(8);
            }
            else if (read_addr == 0x2b && read_len == 8) // return accelerometer data
            {
              uint8_t i;
              volatile uint8_t read_slot = g_accel_write_slot ? 0 : 1;
              for (i = 0; i < 8; i++)
                g_tx_pkt[TX_DATA_START+i] = g_accel_data[read_slot][i];
              send_packet(8);
            }
            else if (read_addr == 0x25 && read_len == 4) // return encoder value
            {
              volatile uint16_t encoder_latch[2];
              cli();
              encoder_latch[0] = TCD0.CNT;
              encoder_latch[1] = TCD1.CNT;
              sei();
              g_tx_pkt[TX_DATA_START  ] = *((uint8_t *)(&encoder_latch[0])  );
              g_tx_pkt[TX_DATA_START+1] = *((uint8_t *)(&encoder_latch[0])+1);
              g_tx_pkt[TX_DATA_START+2] = *((uint8_t *)(&encoder_latch[1])  );
              g_tx_pkt[TX_DATA_START+3] = *((uint8_t *)(&encoder_latch[1])+1);
              send_packet(4);
            }
          }
          else if (g_rx_instruction == 0x03)
          {
            if (g_rx_length == 7) // 32-bit write
            {
              const uint16_t val_1 = ((uint16_t)g_rx_param[1]) | ((uint16_t)(g_rx_param[2]&0x7f) << 8);
              const uint16_t val_2 = ((uint16_t)g_rx_param[3]) | ((uint16_t)(g_rx_param[4]&0x7f) << 8);
              if (g_rx_param[0] == 0x20) // target velocity (dynamixel "moving speed")
              {
                g_tgt_vel[0] = val_1;
                g_tgt_vel[1] = val_2;
                if (val_1 > 0) // set (or retain) timer clock
                {
                  TCC0.CTRLA = TC_CLKSEL_DIV64_gc;
                  if (val_1 > MIN_TIMER_TOP)
                    TCC0.CCABUF = val_1;
                  else
                    TCC0.CCABUF = MIN_TIMER_TOP;
                }
                else // stop timer clock
                  TCC0.CTRLA = TC_CLKSEL_OFF_gc;

                if (val_2 > 0) // set (or retain) timer clock
                {
                  TCE0.CTRLA = TC_CLKSEL_DIV64_gc;
                  if (val_2 > MIN_TIMER_TOP)
                    TCE0.CCABUF = val_2;
                  else
                    TCE0.CCABUF = MIN_TIMER_TOP;
                }
                else // stop timer clock
                  TCE0.CTRLA = TC_CLKSEL_OFF_gc;
/*
                if (g_rx_param[2] & 0x80)
                  PORTC.OUTSET = PIN2_bm;
                else
                  PORTC.OUTCLR = PIN2_bm;

                if (g_rx_param[4] & 0x80)
                  PORTE.OUTSET = PIN1_bm;
                else
                  PORTE.OUTCLR = PIN1_bm;
*/
                g_motor_dir[0] = g_rx_param[2] & 0x80;
                g_motor_dir[1] = g_rx_param[4] & 0x80;
              }
            }
            else if (g_rx_length == 5) // 16-bit write
            {
              const uint8_t val_1 = g_rx_param[1], val_2 = g_rx_param[2];
              if (g_rx_param[0] == 0x18) // torque on/off
              {
                g_motor_enabled[0] = val_1;
                g_motor_enabled[1] = val_2;
                if (g_motor_enabled[0])
                  PORTC.OUTSET = PIN1_bm;
                else
                  PORTC.OUTCLR = PIN1_bm;
                if (g_motor_enabled[1])
                  PORTE.OUTSET = PIN2_bm;
                else
                  PORTE.OUTCLR = PIN2_bm;
              }
            }
            else if (g_rx_length == 4) // 8-bit write
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

// harrr count yer steps
ISR(TCC0_CCA_vect)
{
  if (PORTC.OUT & 0x01) // leading edge: latch for step count
  {
    if (PORTC.OUT & 0x04)
      g_motor_pos[0]++;
    else
      g_motor_pos[0]--;
  }
  else                  // trailing edge: set next step direction
  {
    // update the direction output pin
    if (g_motor_dir[0])
      PORTC.OUTSET = PIN2_bm;
    else
      PORTC.OUTCLR = PIN2_bm;
  }
}

ISR(TCE0_CCA_vect)
{
  if (PORTE.OUT & 0x01) // leading edge: latch for step count
  {
    if (PORTE.OUT & 0x02)
      g_motor_pos[1]++;
    else
      g_motor_pos[1]--;
  }
  else                  // trailing edge: set next step direction
  {
    if (g_motor_dir[1])
      PORTE.OUTSET = PIN1_bm;
    else
      PORTE.OUTCLR = PIN1_bm;
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
  uint8_t i;

  OSC.CTRL |= OSC_RC32MEN_bm;
  while (!(OSC.STATUS & OSC_RC32MRDY_bm)) { } // spin until we settle
  CCP = CCP_IOREG_gc;
  CLK.CTRL = 0x01;

  PORTC.DIRSET = PIN0_bm | PIN1_bm | PIN2_bm;
  PORTE.DIRSET = PIN0_bm | PIN1_bm | PIN2_bm;

  TCC0.CTRLA = TC_CLKSEL_OFF_gc;
  TCE0.CTRLA = TC_CLKSEL_OFF_gc;

  TCC0.CTRLB = TC_WGMODE_FRQ_gc | TC0_CCAEN_bm;
  TCE0.CTRLB = TC_WGMODE_FRQ_gc | TC0_CCAEN_bm;

  TCC0.INTCTRLB = TC_CCAINTLVL_MED_gc;
  TCE0.INTCTRLB = TC_CCAINTLVL_MED_gc;

  for (i = 0; i < 2; i++)
  {
    g_tgt_vel[i] = 0;
    g_cur_vel[i] = 0;
    g_motor_enabled[i] = 0;
    g_motor_pos[i] = 0;
  }

  uart_init();

  PMIC.CTRL |= PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm; // enable low-level interrupts

  PORTD.DIRSET = PIN4_bm | PIN5_bm | PIN7_bm;
  PORTD.OUTSET = PIN4_bm | PIN7_bm;
  SPID.CTRL = SPI_ENABLE_bm | SPI_MODE0_bm | SPI_MODE1_bm | SPI_MASTER_bm | SPI_PRESCALER_DIV64_gc;

  PORTC.DIRCLR = PIN3_bm | PIN4_bm | PIN5_bm;
  PORTD.DIRCLR = PIN0_bm | PIN1_bm | PIN2_bm;

  PORTC.PIN3CTRL = PORT_ISC_BOTHEDGES_gc; // quadrature index
  PORTC.PIN4CTRL = PORT_ISC_LEVEL_gc; // quadrature A
  PORTC.PIN5CTRL = PORT_ISC_LEVEL_gc; // quadrature B

  PORTD.PIN0CTRL = PORT_ISC_BOTHEDGES_gc; // quadrature index
  PORTD.PIN1CTRL = PORT_ISC_LEVEL_gc; // quadrature A
  PORTD.PIN2CTRL = PORT_ISC_LEVEL_gc; // quadrature B

  EVSYS.CH0MUX = EVSYS_CHMUX_PORTD_PIN1_gc; // A & B inputs to quadrature decoder
  EVSYS.CH1MUX = EVSYS_CHMUX_PORTD_PIN0_gc; // index input to quadrature decoder
  EVSYS.CH0CTRL = EVSYS_QDEN_bm | EVSYS_QDIEN_bm | EVSYS_DIGFILT_2SAMPLES_gc;
  EVSYS.CH1CTRL = EVSYS_DIGFILT_2SAMPLES_gc;

  EVSYS.CH2MUX = EVSYS_CHMUX_PORTC_PIN4_gc; // A & B inputs to quadrature decoder
  EVSYS.CH3MUX = EVSYS_CHMUX_PORTC_PIN3_gc; // index input to quadrature decoder
  EVSYS.CH2CTRL = EVSYS_QDEN_bm | EVSYS_QDIEN_bm | EVSYS_DIGFILT_2SAMPLES_gc;
  EVSYS.CH3CTRL = EVSYS_DIGFILT_2SAMPLES_gc;
  
  TCD0.CTRLD = TC_EVACT_QDEC_gc | TC_EVSEL_CH0_gc;
  TCD0.PER = 2500 * 4 - 1;
  TCD0.CTRLA = TC_CLKSEL_DIV1_gc;

  TCD1.CTRLD = TC_EVACT_QDEC_gc | TC_EVSEL_CH2_gc;
  TCD1.PER = 2500 * 4 - 1;
  TCD1.CTRLA = TC_CLKSEL_DIV1_gc;
  sei();

  while(1)
  {
    _delay_ms(10);
    poll_accel();
  }
  return 0; // or not
}  
