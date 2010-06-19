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
#include "openarms/GripperPosition.h"
#include "openarms/GripperSensors.h"

LightweightSerial *g_serial = NULL; 
const size_t NUM_MOTORS = 5;

bool process_byte(uint8_t b, ros::Publisher *pub)
{
  static uint8_t pkt[256];
  static uint8_t write_pos = 0, expected_len = 0, s_source = 0;
  static enum { IDLE, LEN, SOURCE, DATA } s_state = IDLE;
  switch (s_state)
  {
    case IDLE:
      if (b == 0xff)
        s_state = LEN;
      else
        printf("%x found but 0xff expected\n", b);
      break;
    case LEN:
      expected_len = b;
      if (expected_len > 50)
        s_state = IDLE; // fail. wait for next start token and try again
      else
        s_state = SOURCE;
      write_pos = 0;
      break;
    case SOURCE:
      s_source = b;
      s_state = DATA;
      break;
    case DATA:
      pkt[write_pos++] = b;
      if (write_pos >= expected_len-1)
      {
        s_state = IDLE;
        if (s_source == 0x01)
        {
          int16_t sensors[3];
          for (int i = 0; i < 3; i++)
            sensors[i] = (pkt[2*i+1] << 8) | pkt[2*i];
          //printf("%6d %6d %6d\n", sensors[0], sensors[1], sensors[2]);
          openarms::GripperSensors msg;
          msg.x = sensors[0];
          msg.y = sensors[1];
          msg.z = sensors[2];
          pub->publish(msg);
          return true;
        }
        else if (s_source == 0x02)
        {
          printf("received %x bytes from %x\n", write_pos, s_source);
        }
      }
      return false; 
      break;
  }
  return false; // didn't send a message
}

void dynamixel_write(uint8_t *data, uint8_t data_len)
{
  uint8_t pkt[50];
  if (data_len > sizeof(pkt)-2)
    data_len = (uint8_t)sizeof(pkt)-2;
  pkt[0] = data_len + 2;
  pkt[1] = 0x01; // write to dynamixel
  for (size_t i = 0; i < data_len; i++)
    pkt[i+2] = data[i];
  pkt[2+data_len] = '\n';
  for (int i = 0; i < 3 + (int)data_len; i++)
  {
    g_serial->write(pkt[i]);
    ros::Duration(0.001).sleep();
  }
}

uint8_t dynamixel_read_8bit(uint8_t addr)
{
  uint8_t pkt[20];
  for (int i = 0; i < 20; i++)
    pkt[i] = 0;
  pkt[0] = 19;
  pkt[1] = 0x05;
  pkt[19] = '\n';
  for (int i = 0; i < 20; i++)
  {
    g_serial->write(pkt[i]);
    ros::Duration(0.001).sleep();
  }
  //g_serial->write_block(pkt, 20);
  uint8_t val = 0;
  return val;
}

uint8_t calc_dynamixel_checksum(uint8_t *pkt, uint8_t pkt_len)
{
  uint8_t sum = 0;
  for (uint8_t i = 2; i < pkt_len; i++)
    sum += pkt[i];
  return ~sum;
}

void set_gripper_goal(uint16_t goal)
{
  uint8_t pkt[20];
  pkt[0] = 0xff;
  pkt[1] = 0xff;
  pkt[2] = 0x02; // servo #2 on our setup
  pkt[3] = 5; // length: 3 + number registers written (two)
  pkt[4] = 0x03; // write_data instruction
  pkt[5] = 0x1e; // address: the "goal position" register
  pkt[6] = (uint8_t)(goal & 0xff);
  pkt[7] = (uint8_t)(goal >> 8);
  pkt[8] = calc_dynamixel_checksum(pkt, 8);
  dynamixel_write(pkt, 9);
}

void set_torque_enable(uint8_t enable)
{
  uint8_t pkt[10];
  pkt[0] = 0xff;
  pkt[1] = 0xff;
  pkt[2] = 0x02;
  pkt[3] = 4;
  pkt[4] = 0x03;
  pkt[5] = 0x18; // torque enable register
  pkt[6] = enable;
  pkt[7] = calc_dynamixel_checksum(pkt, 7);
  dynamixel_write(pkt, 8);
}

void position_cb(const openarms::GripperPosition::ConstPtr &msg)
{
  /*
  if (msg->mode.size() != msg->vel.size() ||
      msg->vel.size()  != msg->pos.size())
  {
    ROS_DEBUG("ahhhhhh mode, vel, and pos vectors must be the same size");
    return;
  }
  uint8_t pkt[NUM_MOTORS*6+1];
  for (size_t i = 0; i < NUM_MOTORS && i < msg->vel.size(); i++)
  {
    pkt[i*6  ] = msg->mode[i]; 
    pkt[i*6+1] = (uint8_t)msg->vel[i];
    pkt[i*6+2] = (uint8_t)(msg->pos[i] >> 24);
    pkt[i*6+3] = (uint8_t)(msg->pos[i] >> 16);
    pkt[i*6+4] = (uint8_t)(msg->pos[i] >>  8);
    pkt[i*6+5] = (uint8_t)(msg->pos[i]      );
  }
  pkt[NUM_MOTORS*6] = '\n';
  g_serial->write_block(pkt, sizeof(pkt));
  printf("wrote %04d bytes ", (int)sizeof(pkt));
  for (size_t i = 0; i < NUM_MOTORS; i++)
    printf(" %x-%03d ", msg->mode[i], msg->vel[i]);
  printf("\n");
  */
  printf("gripper position msg %d\n", msg->position);
  set_gripper_goal(330+((float)msg->position/255.0*300.0));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "oa2_stepper");
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
  ros::Subscriber sub = n.subscribe("gripper_position", 1, position_cb);
  ros::Publisher  pub = n.advertise<openarms::GripperSensors>("gripper_sensors",1);
  //ros::Time t_prev = ros::Time::now();
  //ros::spin();
  
  ros::Time last_dynamixel_query_time = ros::Time::now();

  set_torque_enable(1);
  while (n.ok())
  {
    uint8_t read_buf[60];
    int nread = s->read_block(read_buf, sizeof(read_buf)-1);
    if (nread)
    {
      for (int i = 0; i < nread; i++)
      {
        //printf("    %d\n", read_buf[i]);
        if (process_byte(read_buf[i], &pub))
          ros::spinOnce();
      }
    }
    else
      ros::Duration(0.00001).sleep();

    if (last_dynamixel_query_time + ros::Duration(0.1) < ros::Time::now())
    {
      last_dynamixel_query_time = ros::Time::now();
      //dynamixel_read_8bit(0x2b);
    }

    ros::spinOnce();
  }
  set_torque_enable(0);

  // shutdown motors before we quit
  /*
  uint8_t pkt[NUM_MOTORS*6+1];
  memset(pkt, 0, sizeof(pkt));
  pkt[NUM_MOTORS*6] = '\n';
  s->write_block(pkt, sizeof(pkt));
  delete s;
  */
  return 0;
}

