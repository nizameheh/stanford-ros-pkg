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
        s_state = DATA;
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
        uint16_t pos = pkt[0] + (uint16_t)(pkt[1] << 8);
        printf("%d: %5d\n", servo_id, pos);
        //printf("packet ok\n");
        //printf("%d byte packet\n", expected_len);
        //pub->publish(msg);
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

void send_torque(uint8_t id, uint16_t torque, uint8_t dir)
{
  // implemented from the robotis user's manual, page 16
  uint8_t pkt[20];
  pkt[0] = 0xff;
  pkt[1] = 0xff;
  pkt[2] = (uint8_t)id;
  pkt[3] = 5; // 3 parameters + 2
  pkt[4] = 0x03; // "write data" instruction
  pkt[5] = 0x20; // "moving speed" register
  pkt[6] = (uint8_t)(torque & 0xff);
  pkt[7] = (uint8_t)((0x3 & (torque >> 8))) | (dir ? 0x04 : 0x00);
  pkt[8] = calc_checksum(pkt, 8);
  g_serial->write_block(pkt, 9);
}

void enable_motors(uint8_t enable)
{
  uint8_t pkt[20];
  pkt[0] = 0xff;
  pkt[1] = 0xff;
  pkt[2] = 0xfe; // broadcast
  pkt[3] = 4; // 2 parameters + 2
  pkt[4] = 0x03; // "write data"
  pkt[5] = 0x18; // torque on/off
  pkt[6] = (enable ? 1 : 0);
  pkt[7] = calc_checksum(pkt, 7);
  g_serial->write_block(pkt, 8);
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
  g_serial->write_block(pkt, 8);
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
  
  while (n.ok())
  {
    uint8_t read_buf[60];
    int nread = s->read_block(read_buf, sizeof(read_buf)-1);
    if (nread)
    {
      for (int i = 0; i < nread; i++)
      {
        printf("    %x %c\n", read_buf[i], read_buf[i]);
        //if (process_byte(read_buf[i])) //, &pub))
        //  ros::spinOnce();
      }
    }
    else
      ros::Duration(0.001).sleep();
    ros::spinOnce();
    // blast status-request packets periodically
    ros::Time t = ros::Time::now();
#if 0
    if ((t - t_prev).toSec() > 0.1)
    {
      for (size_t i = 0; i < 1 /*NUM_MOTORS*/; i++)
        query_motor(i);
      t_prev = t;
    }
#endif
  }
  //enable_motors(0); // turn em off
  delete s;
  return 0;
}

