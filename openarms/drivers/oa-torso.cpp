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
#include "openarms/StepperTarget.h"
#include "openarms/ArmSensors.h"

LightweightSerial *g_serial = NULL; 
const size_t NUM_MOTORS = 4;

bool process_byte(uint8_t b, ros::Publisher *pub)
{
  static uint8_t pkt[256];
  static uint8_t write_pos = 0, expected_len = 0;
  static enum { IDLE, LENGTH, DATA, CHECKSUM } s_state = IDLE;
  switch (s_state)
  {
    case IDLE:
      if (b == 0xfe)
        s_state = LENGTH;
      else
        printf("%x found but 0xfe expected\n", b);
      break;
    case LENGTH:
      expected_len = b;
      if (expected_len > 20)
      {
        printf("bogus expected length: %x\n", b);
        s_state = IDLE; // fail. wait for next sentinel.
      }
      else
        s_state = DATA;
      write_pos = 0;
      break;
    case DATA:
      pkt[write_pos++] = b;
      if (write_pos >= expected_len)
        s_state = CHECKSUM;
      break;
    case CHECKSUM:
      s_state = IDLE;
      uint8_t local_csum = expected_len;
      for (int i = 0; i < expected_len; i++)
        local_csum += pkt[i];
      if (local_csum == b)
      {
        openarms::ArmSensors msg;
        msg.pos.resize(2);
        msg.pos[0] = *((int32_t *)&pkt[0]);
        msg.pos[1] = *((int32_t *)&pkt[4]);
        //printf("%10d %10d\n", msg.pos[0], msg.pos[1]);
        pub->publish(msg);
        return true;
      }
      else
        printf("checksum fail: local sum=%x wire sum=%x\n", local_csum, b);
      return false; // sent a message
      break;
  }
  return false; // didn't send a message
}

void step_cb(const openarms::StepperTarget::ConstPtr &msg)
{
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
  /*
  printf("wrote %04d bytes ", (int)sizeof(pkt));
  for (size_t i = 0; i < NUM_MOTORS; i++)
    printf(" %x-%03d ", msg->mode[i], msg->vel[i]);
  printf("\n");
  */
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "oa_torso");
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
  ros::Subscriber sub = n.subscribe("stepper_targets", 1, step_cb);
  ros::Publisher pub = n.advertise<openarms::ArmSensors>("arm_sensors", 1);
  //ros::Time t_prev = ros::Time::now();
  //ros::spin();
  
  while (n.ok())
  {
    uint8_t read_buf[60];
    int nread = s->read_block(read_buf, sizeof(read_buf)-1);
    if (nread)
    {
      for (int i = 0; i < nread; i++)
      {
        //printf("    %d  %x\n", read_buf[i], read_buf[i]);
        if (process_byte(read_buf[i], &pub))
          ros::spinOnce();
      }
      //printf("\n");
    }
    else
      ros::Duration(0.001).sleep();
    ros::spinOnce();
  }

  // shutdown motors before we quit
  uint8_t pkt[NUM_MOTORS*6+1];
  memset(pkt, 0, sizeof(pkt));
  pkt[NUM_MOTORS*6] = '\n';
  s->write_block(pkt, sizeof(pkt));
  delete s;
  return 0;
}

