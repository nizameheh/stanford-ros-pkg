// copyright 2010 morgan quigley, mquigley@cs.stanford.edu
// bsd license blah blah

#include <stdlib.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <avr/interrupt.h>

// hardware connections (completely arbitrary)
// PA0 = ~enable0  (io 22 on arduino mega)
// PA1 = ~enable1  (io 23 on arduino mega)
// PB7 =  led (on arduino mega)
// PC0 =  dir0   (io 37 on arduino mega)
// PC1 =  dir1   (io 36 on arduion mega)
// PB5 =  OC1A = step0 (pwm 11 on arduino mega)
// PE3 =  OC3A = step1 (pwm  5 on arduino mega)

#define NUM_MOTORS 4
#define CMD_PKT_LEN (NUM_MOTORS*6+1)
#define MODE_FLOAT    0
#define MODE_HOLD     1
#define MODE_VELOCITY 2
#define MODE_POSITION 3

#define TX_PKT_LEN 11

volatile uint8_t  g_cmd_pkt[CMD_PKT_LEN+1], g_cmd_pkt_write_pos = 0;
volatile uint8_t  g_motor_mode[NUM_MOTORS];
volatile uint8_t  g_motor_dir;
volatile uint8_t  g_motor_vel[NUM_MOTORS];
volatile int32_t  g_motor_tgt[NUM_MOTORS];
volatile uint8_t  g_tgt_update[NUM_MOTORS];
volatile uint32_t g_motor_pos[NUM_MOTORS];
volatile uint8_t  g_tx_pkt[TX_PKT_LEN], g_tx_pkt_read_pos = 0;
volatile uint8_t  g_timer_flag = 0;


ISR(USART0_RX_vect)
{
  volatile uint8_t b = UDR0;
  if (g_cmd_pkt_write_pos < CMD_PKT_LEN)
    g_cmd_pkt[g_cmd_pkt_write_pos++] = b;
  if (b == '\n')
  {
    if (g_cmd_pkt_write_pos == CMD_PKT_LEN)
    {
      volatile uint8_t m = 0;
      for (m = 0; m < NUM_MOTORS; m++)
      {
        g_motor_mode[m] = g_cmd_pkt[m*6  ] & 0x7f;
        if (g_motor_mode[m] == MODE_VELOCITY)
        {
          g_motor_vel[m] = g_cmd_pkt[m*6+1];
          /*
          if (g_motor_vel[m] > 248)
            g_motor_vel[m] = 248;
          */
          if (g_cmd_pkt[m*6] > 128)
            g_motor_dir |= (1 << m);
          else
            g_motor_dir &= ~(1 << m);
        }
        else if (g_motor_mode[m] == MODE_POSITION)
          g_motor_tgt[m] = *((uint32_t *)&g_cmd_pkt[m*6+2]);
        g_tgt_update[m] = 1;
      }
      PORTB ^= 0x80;
      g_cmd_pkt_write_pos = 0;
    }
    else if (g_cmd_pkt_write_pos >= CMD_PKT_LEN)
      g_cmd_pkt_write_pos = 0;
  }
}

ISR(USART0_TX_vect)
{
  if (g_tx_pkt_read_pos < TX_PKT_LEN)
    UDR0 = g_tx_pkt[g_tx_pkt_read_pos++];
}

ISR(TIMER1_COMPA_vect)
{
  if (PORTC & 0x01)
    g_motor_pos[0]++;
  else
    g_motor_pos[0]--;
}

ISR(TIMER3_COMPA_vect)
{
  if (PORTC & 0x02)
    g_motor_pos[1]++;
  else
    g_motor_pos[1]--;
}

ISR(TIMER4_COMPA_vect)
{
  g_timer_flag = 1;
}

uint8_t vel_to_ocr(uint8_t vel)
{
  // 1024 timer divisor @ 16 MHz clock, the timer is running at 16 KHz
  // vel here means "X steps per second" where x is 5 or whatever
  if (vel == 0)
    return 255; // not really
  uint16_t ocr = 2000 / (uint16_t)vel;
  //uint16_t ocr = 16384 / (uint16_t)vel;
  if (ocr > 255) // too slow
    ocr = 255;
  if (ocr < 2)   // too fast
    ocr = 2;
  return (uint8_t)ocr; // just right
}

void send_tx_pkt()
{
  uint8_t csum = 0, i;
  g_tx_pkt[0] = 0xfe;
  g_tx_pkt[1] = 8;
  *((uint32_t *)(g_tx_pkt+2)) = g_motor_pos[0];
  *((uint32_t *)(g_tx_pkt+6)) = g_motor_pos[1];
  g_tx_pkt_read_pos = 1;
  for (i = 1; i < 10; i++)
    csum += g_tx_pkt[i];
  g_tx_pkt[10] = csum;
  UDR0 = g_tx_pkt[0];
}

int main(void)
{
  uint8_t m;
  wdt_disable();

  // wgm = 0100, divide clock by 1024, toggle when hit OCRnA
  TCCR1A = 0x00;
  TCCR1B = _BV(WGM12) | _BV(CS12) | _BV(CS10);
  TCCR3A = 0x00;
  TCCR3B = _BV(WGM32) | _BV(CS32) | _BV(CS30);

  // timer to know when to blast back the status messages
  TCCR4A = 0;
  TCCR4B = _BV(WGM42) | _BV(CS42) | _BV(CS40);

  TIMSK1 = _BV(OCIE1A);
  TIMSK3 = _BV(OCIE3A);
  TIMSK4 = _BV(OCIE4A);

  DDRA = 0x03;
  DDRB = 0xa0;
  DDRC = 0x03;
  DDRE = 0x08;
 
  PORTA = 0x03;
  PORTB = 0x00;
  PORTC = 0x00;
  PORTE = 0x00;

  OCR1A = 0xf0;
  OCR3A = 0xf0;
  OCR4A = 0x009b; // delay between tx packets, units of X/16khz

  UBRR0  = 0; // 1 megabaud
  UCSR0A = 0;
  UCSR0B = _BV(RXEN0)  | _BV(RXCIE0) | _BV(TXEN0) | _BV(TXCIE0);
  UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); // 8n1
  sei();

  g_motor_dir = 0;
  for (m = 0; m < NUM_MOTORS; m++)
  {
    g_motor_mode[m] = 0;
    g_motor_vel[m] = 0;
    g_motor_tgt[m] = 0;
    g_tgt_update[m] = 0;
  }

  //OCR1A = 0xf0;
  //TCCR1A = 0x40;
  m = 0;
  while (1)
  {
    if (g_timer_flag)
    {
      g_timer_flag = 0;
      send_tx_pkt();
    }

    if (g_tgt_update[0] &&  // update and we're at rollover or clock stopped
        ((TCNT1 == 0 || TCCR1A == 0x00) || !(TCCR1B & _BV(CS12))))
    {
      g_tgt_update[0] = 0;
      if (g_motor_dir & 0x01)
        PORTC |= 0x01;
      else
        PORTC &= ~0x01;
      OCR1A = vel_to_ocr(g_motor_vel[0]);
      if (g_motor_vel[0] == 0)
      {
        TCCR1A = 0x00;
        TCCR1B &= ~(_BV(CS12) | _BV(CS10)); // stop timer clock
      }
      else
      {
        TCCR1A = 0x40;
        TCCR1B |= _BV(CS12) | _BV(CS10);
      }
    }
    if (g_tgt_update[1] &&
        ((TCNT3 == 0 || TCCR3A == 0x00) || !(TCCR3B & _BV(CS32))))
    {
      g_tgt_update[1] = 0;
      if (g_motor_dir & 0x02)
        PORTC |= 0x02;
      else
        PORTC &= ~0x02;
      OCR3A = vel_to_ocr(g_motor_vel[1]);
      if (g_motor_vel[1] == 0)
      {
        TCCR3A = 0x00;
        TCCR3B &= ~(_BV(CS32) | _BV(CS30)); // stop timer clock
      }
      else
      {
        TCCR3A = 0x40;
        TCCR3B |= _BV(CS32) | _BV(CS30);
      }
    }
  }
}

