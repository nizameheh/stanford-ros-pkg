// copyright 2009 Morgan Quigley, mquigley@cs.stanford.edu
// bsd license blah blah

#include <unistd.h>
#include <cstdio>
#include <ctime>
#include <cstdlib>
#include <string>
#include <csignal>
#include "serial_port/lightweightserial.h"
#include "ros/time.h"
#include "ros/ros.h"
#include "openarms/ArmActuators.h"
#include "openarms/ArmSensors.h"

LightweightSerial *g_serial = NULL; 
const size_t NUM_MOTORS = 3;
enum rx_state_t { PING, STEPPER_POS, SERVO_POS, ACCELEROMETER } g_rx_state;
// the servos are assumed to be programmed as 0,1,..,NUM_MOTORS

bool process_byte(uint8_t b) //, ros::Publisher *pub)
{
  static uint8_t pkt[256], servo_id = 0, s_err = 0;
  static uint8_t write_pos = 0, expected_len = 0;
  static enum { PREAMBLE1, PREAMBLE2, ID, LEN, ERR, DATA, CHECKSUM } s_state;
  switch (s_state)
  {
    case PREAMBLE1:
      if (b == 0xff)
        s_state = PREAMBLE2;
      else
        printf("%x found but 0xff expected\n", b);
      break;
    case PREAMBLE2:
      if (b == 0xff)
        s_state = ID;
      else
      {
        printf("%x found but 0xff expected\n", b);
        s_state = PREAMBLE1; // try to re-sync
      }
      break;
    case ID:
      servo_id = b;
      s_state = LEN;
      break;
    case LEN:
      expected_len = b;
      if (expected_len > 20)
      {
        s_state = PREAMBLE1; // fail. try to re-sync.
        printf("expected a length, but got %x\n", b);
      }
      else
        s_state = ERR;
      write_pos = 0;
      break;
    case ERR:
      s_err = b;
      if (!s_err)
      {
        if (expected_len > 2)
          s_state = DATA;
        else
          s_state = CHECKSUM;
      }
      else
        s_state = CHECKSUM;
      break;
    case DATA:
      pkt[write_pos++] = b;
      if (write_pos >= expected_len-2)
        s_state = CHECKSUM;
      break;
    case CHECKSUM:
      s_state = PREAMBLE1;
      uint8_t local_csum = servo_id + expected_len + s_err;
      //printf("id = %d  len = %d  err = %x\n", servo_id, expected_len, s_err);
      for (int i = 0; !s_err && i < expected_len-2; i++)
        local_csum += pkt[i];
      local_csum = ~local_csum;
      if (local_csum == b)
      {
        printf("packet from %d\n", servo_id);
        if (g_rx_state == STEPPER_POS && expected_len == 8+2)
        {
          uint32_t pos_0 = (uint32_t) pkt[0]        + 
                           (uint32_t)(pkt[1] << 8)  +
                           (uint32_t)(pkt[2] << 16) +
                           (uint32_t)(pkt[3] << 24);
          uint32_t pos_1 = (uint32_t) pkt[4]        + 
                           (uint32_t)(pkt[5] << 8)  +
                           (uint32_t)(pkt[6] << 16) +
                           (uint32_t)(pkt[7] << 24);
          printf("%d: %10u %10u\n", servo_id, pos_0, pos_1);
        }
        else if (g_rx_state == SERVO_POS)
        {
          uint16_t pos = pkt[0] + (uint16_t)(pkt[1] << 8);
          printf("%d: %5d\n", servo_id, pos);
        }
        else if (g_rx_state == ACCELEROMETER)
        {
          for (int i = 0; i < 8; i++)
            printf("  %x ", pkt[i]);
          printf("\n");
        }
        else if (g_rx_state == PING)
        {
        }
        /*
        //printf("packet ok\n");
        //printf("%d byte packet\n", expected_len);
        //pub->publish(msg);
        */
        return true;
      }
      else
        printf("checksum fail: local sum=%x wire sum=%x\n", local_csum, b);
      return false; // sent a message
      break;
  }
  return false; // didn't send a message
}

uint8_t calc_checksum(uint8_t *pkt, uint8_t pkt_len)
{
  uint8_t checksum = 0;
  for (int j = 2; j < pkt_len; j++)
    checksum += pkt[j];
  return ~checksum;
}

void write_data(uint8_t id, uint8_t addr, uint8_t length, uint8_t *data)
{
  uint8_t pkt[300];
  pkt[0] = 0xff;
  pkt[1] = 0xff;
  pkt[2] = id;
  pkt[3] = length + 3;
  pkt[4] = 0x03; // "write data"
  pkt[5] = addr;
  for (int i = 0; i < length; i++)
    pkt[6+i] = data[i];
  pkt[6+length] = calc_checksum(pkt, 6+length);
  g_serial->write_block(pkt, 7+length);
}

void send_torque(uint8_t id, uint16_t torque, uint8_t dir)
{
  // implemented from the robotis user's manual, page 16
  uint8_t data[20];
  data[0] = (uint8_t)(torque & 0xff);
  data[1] = (uint8_t)((0x3 & (torque >> 8))) | (dir ? 0x04 : 0x00);
  write_data(id, 0x20, 2, data); // "moving speed" register
}

void send_stepper_vel(uint8_t id, uint16_t vel_0, uint16_t vel_1)
{
  uint8_t data[20];
  data[0] = (uint8_t)(vel_0 & 0xff);
  data[1] = (uint8_t)(vel_0 >> 8);
  data[2] = (uint8_t)(vel_1 & 0xff);
  data[3] = (uint8_t)(vel_1 >> 8);
  write_data(id, 0x20, 4, data);
}

void enable_motors(uint8_t enable)
{
  uint8_t data = (enable ? 1 : 0);
  write_data(0xfe, 0x18, 1, &data);
}

void enable_led(uint8_t id, uint8_t enable)
{
  uint8_t data = (enable ? 1 : 0);
  //printf("set led on motor %x to %d\n", id, data);
  write_data(id, 0x19, 1, &data);
}

void query_motor(uint8_t id)
{
  uint8_t pkt[20];
  pkt[0] = 0xff;
  pkt[1] = 0xff;
  pkt[2] = id;
  pkt[3] = 4; // 2 parameters + 2
  pkt[4] = 0x02; // "read data"
  pkt[5] = 0x24; // present position
  pkt[6] = 2; // read LSB and MSB
  pkt[7] = calc_checksum(pkt, 7);
  g_rx_state = SERVO_POS;
  g_serial->write_block(pkt, 8);
}

void query_stepper_positions(uint8_t id)
{
  uint8_t pkt[20];
  pkt[0] = 0xff;
  pkt[1] = 0xff;
  pkt[2] = id;
  pkt[3] = 4; // 2 parameters + 2
  pkt[4] = 0x02; // "read data"
  pkt[5] = 0x24; // present position
  pkt[6] = 8; // 32 bits from both steppers
  pkt[7] = calc_checksum(pkt, 7);
  g_rx_state = STEPPER_POS;
  g_serial->write_block(pkt, 8);
}

void query_accelerometer(uint8_t id)
{
  uint8_t pkt[20];
  pkt[0] = 0xff;
  pkt[1] = 0xff;
  pkt[2] = id;
  pkt[3] = 4; // 2 parameters + 2
  pkt[4] = 0x02; // "read data"
  pkt[5] = 0x2b; // "temperature" aka accelerometer
  pkt[6] = 8; // 8 bytes for now (version, X, Y, Z, temperature)
  pkt[7] = calc_checksum(pkt, 7);
  g_rx_state = ACCELEROMETER;
  g_serial->write_block(pkt, 8);
}

void ping_motor(uint8_t id)
{
  uint8_t pkt[20];
  pkt[0] = 0xff;
  pkt[1] = 0xff;
  pkt[2] = id;
  pkt[3] = 2; // 0 parameters + 2 overhead
  pkt[4] = 0x01; // ping instruction
  pkt[5] = calc_checksum(pkt, 5);
  printf("pinging motor %d, checksum %x\n", id, pkt[5]);
  g_rx_state = PING;
  g_serial->write_block(pkt, 6);
}

/*
void wrist_torques_cb(const openarms::ArmActuators::ConstPtr &msg)
{
  if (msg->torques.size() != NUM_MOTORS)
  {
    ROS_DEBUG("ahhhhhh was expecting %d torques\n", (int)NUM_MOTORS);
    return;
  }
  // blast a packet to each servo 
  for (size_t i = 0; i < NUM_MOTORS; i++)
  {
    double abs_torque = fabs(msg->torques[i] * 1024.0);
    if (abs_torque >= 1024)
      abs_torque = 1023;
    send_torque(i, (uint16_t)floor(abs_torque), (uint8_t)(msg->torques[i] > 0));
  }
}
*/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "armbus");
  ros::NodeHandle n;
  ros::NodeHandle n_private("~");
  std::string port("/dev/ttyUSB0");
  n_private.getParam("port", port);
  LightweightSerial *s = new LightweightSerial(port.c_str(), 1000000);
  if (!s)
  {
    ROS_FATAL("couldn't open port %s\n", port.c_str());
    ROS_BREAK();
  }
  g_serial = s;
  //ros::Subscriber sub = n.subscribe("wrist_torques", 1, wrist_torques_cb);
  ros::Time t_prev = ros::Time::now();
  //ros::spin();
  //enable_motors(1);
  
  bool even = false;
  uint32_t scheduled = 0;
  bool replied = false;
  const uint32_t SCH_BEGIN           = 0;
  const uint32_t SCH_STEPPER_POS_0   = 0;
  const uint32_t SCH_STEPPER_ACCEL_0 = 1;
  const uint32_t SCH_END             = 1;
  while (n.ok())
  {
    uint8_t read_buf[60];
    int nread = s->read_block(read_buf, sizeof(read_buf)-1);
    if (nread)
    {
      for (int i = 0; i < nread; i++)
      {
        //printf("    %x\n", read_buf[i]);
        if (process_byte(read_buf[i])) //, &pub))
        {
          replied = true;
          ros::spinOnce();
        }
      }
    }
    else
      ros::Duration(0.001).sleep();
    ros::spinOnce();
    // blast status-request packets periodically
    ros::Time t = ros::Time::now();
    if ((t - t_prev).toSec() > 0.1 || replied)
    {
      replied = false;
      enable_led(10, even ? 1 : 0);
      even = !even;
      send_stepper_vel(10, 0x10, 0xffff);
      if (scheduled == SCH_STEPPER_POS_0)
        query_stepper_positions(10);
      else if (scheduled == SCH_STEPPER_ACCEL_0)
        query_accelerometer(10);
      if (++scheduled > SCH_END)
        scheduled = SCH_BEGIN;
      t_prev = t;

#if 0
      for (size_t i = 0; i < 1 /*NUM_MOTORS*/; i++)
        query_motor(i);
#endif
    }
  }
  //enable_motors(0); // turn em off
  delete s;
  return 0;
}

