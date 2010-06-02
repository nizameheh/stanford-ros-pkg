#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define PKT_MAX_LEN 50
static volatile uint8_t g_servo_tx_pkt[PKT_MAX_LEN];
static volatile uint8_t g_servo_tx_pkt_read_pos = 0, g_servo_tx_pkt_len;
static volatile uint8_t g_servo_rx_pkt[PKT_MAX_LEN]; // offset and length in first 2
static volatile uint8_t g_servo_rx_pkt_write_pos = 0, g_servo_rx_pkt_ready = 0;
static volatile uint8_t g_host_rx_pkt[PKT_MAX_LEN];
static volatile uint8_t g_host_rx_pkt_write_pos = 0, g_host_rx_pkt_expected_len = 1;

void dynamixel_write(uint8_t addr, uint8_t *data, uint8_t data_len)
{
  uint8_t i;
  g_servo_tx_pkt[0] = 0xff;
  g_servo_tx_pkt[1] = 0xff;
  g_servo_tx_pkt[2] = 0x02;
  g_servo_tx_pkt[3] = 0x04; // 3 bytes overhead, two bytes of parameter
  g_servo_tx_pkt[4] = 0x03; // instruction 0x03: write data
  g_servo_tx_pkt[5] = addr; // goal address
  for (i = 0; i < data_len; i++)
    g_servo_tx_pkt[i] = data[i];
}

void dynamixel_read(uint8_t addr, uint8_t data_len)
{
  g_servo_tx_pkt[0] = 0xff;
  g_servo_tx_pkt[1] = 0xff;
  g_servo_tx_pkt[2] = 0x02;
  g_servo_tx_pkt[3] = 0x04; // read_data always has length 4
  g_servo_tx_pkt[4] = 0x04; // instruction 0x03: read data
  g_servo_tx_pkt[5] = addr; // goal address
  g_servo_tx_pkt[6] = data_len;
}

ISR(USART1_TX_vect)
{
  if (g_servo_tx_pkt_read_pos < g_servo_tx_pkt_len)
    UDR1 = g_servo_tx_pkt[g_servo_tx_pkt_read_pos++];
}


ISR(USART0_RX_vect)
{
  volatile uint8_t b = UDR0;
  if (g_host_rx_pkt_write_pos == 0xff)
  {
    g_host_rx_pkt_expected_len = b;
    g_host_rx_pkt_write_pos = 0;
  }
  else if (g_host_rx_pkt_write_pos < PKT_MAX_LEN && 
           g_host_rx_pkt_write_pos < g_host_rx_pkt_expected_len)
    g_host_rx_pkt[g_host_rx_pkt_write_pos++] = b;

  if (b == '\n')
  {
    if (g_host_rx_pkt_write_pos == g_host_rx_pkt_expected_len)
    {
      if (g_host_rx_pkt[0] == 1)
      {
        uint8_t i;
        for (i = 1; i < g_host_rx_pkt_write_pos; i++)
          g_servo_tx_pkt[i-1] = g_host_rx_pkt[i];
        g_servo_tx_pkt_len = g_host_rx_pkt_write_pos-1;
        g_servo_tx_pkt_read_pos = 1;
        UDR1 = g_servo_tx_pkt[0];
        PORTD ^= 0x10; // blink light        
      }
      g_host_rx_pkt_write_pos = 0xff;
    }
    else 
    {
      if (g_host_rx_pkt_write_pos >= PKT_MAX_LEN)
        g_host_rx_pkt_write_pos = 0xff;
    }
  }
}

void send_packet_to_host(uint8_t *pkt)
{
  uint8_t pkt_remain = 0, *pkt_send_pos = 0;
  pkt_send_pos = &pkt[0];
  while (!(UCSR0A & _BV(UDRE0))) { }
  for (pkt_remain = pkt[1]+2; pkt_remain > 0; pkt_remain--)
  {
    UDR0 = *pkt_send_pos++;
    while (!(UCSR0A & _BV(UDRE0))) { }
  }
}

int main(int argc, char **argv)
{
  //uint16_t x = 0, y = 0, z = 0;
  uint8_t pkt[60], spi_reg_idx;

  DDRD  = 0x1a; // drive PD1, PD3, PD4
  PORTD = 0x14; // enable pullup on PD2 (rxd1)
  DDRB  = 0xb2; // drive MOSI, SCK, SS
  PORTB = 0xbf; // idle everybody high

  UBRR0 = 0; // 1 megabit
  UCSR0A = _BV(U2X0);
  UCSR0B = _BV(TXEN0) | _BV(RXEN0) | _BV(RXCIE0);
  UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); // 8n1

  UBRR1 = 0;
  UCSR1A = _BV(U2X0);
  UCSR1B = _BV(TXEN0) | _BV(TXCIE1);
  UCSR1C = _BV(UCSZ11) | _BV(UCSZ10); // 8n1

  SPCR = _BV(SPE) | _BV(MSTR) | _BV(CPHA) | _BV(CPOL);

  sei();

  pkt[0] = 0xff;

  while (1)
  {
    _delay_ms(1);
    // query first four registers
    PORTB &= ~0x02; // pull down SS
    _delay_us(1);
    SPDR = 0x82; // set read address to 2
    while (!(SPSR & _BV(SPIF))) { }
    SPDR = 0x00; // dummy
    for (spi_reg_idx = 2; spi_reg_idx < 8; spi_reg_idx++)
    {
      while (!(SPSR & _BV(SPIF))) { }
      pkt[spi_reg_idx-2+3] = SPDR;
      if (spi_reg_idx < 7)
        SPDR = 0; // dummy
    }
    _delay_us(1);
    PORTB |= 0x02;

    pkt[1] = 7;
    pkt[2] = 0x01; // accelerometer packet
    send_packet_to_host(pkt);

    if (g_servo_rx_pkt_ready)
    {
      uint8_t read_pos;
      pkt[1] = g_servo_rx_pkt_write_pos+1;
      pkt[2] = 0x02; // servo packet
      for (read_pos = 0; read_pos < g_servo_rx_pkt_write_pos; read_pos++)
        pkt[read_pos+3] = g_servo_rx_pkt[read_pos];
      send_packet_to_host(pkt);
      g_servo_rx_pkt_ready = 0;
      g_servo_rx_pkt_write_pos = 0;
    }
  }
  
  return 0;
}
