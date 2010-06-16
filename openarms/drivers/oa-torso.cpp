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

LightweightSerial *g_serial = NULL; 
const size_t NUM_MOTORS = 4;

#if 0
bool process_byte(uint8_t b, ros::Publisher *pub)
{
  static uint8_t pkt[256];
  static uint8_t write_pos = 0, expected_len = 0;
  static enum { IDLE, DATA, CHECKSUM, TERMINAL } s_state = TERMINAL;
  switch (s_state)
  {
    case TERMINAL:
      if (b == '\n')
        s_state = IDLE;
      else
        printf("%x found but 0x0a expected\n", b);
      break;
    case IDLE:
      expected_len = b;
      if (expected_len > 100)
        s_state = TERMINAL; // fail. wait for next newline and try again
      else
        s_state = DATA;
      write_pos = 0;
      break;
    case DATA:
      pkt[write_pos++] = b;
      if (write_pos >= expected_len-2)
        s_state = CHECKSUM;
      break;
    case CHECKSUM:
      s_state = TERMINAL;
      uint8_t local_csum = expected_len;
      for (int i = 0; i < expected_len-2; i++)
        local_csum += pkt[i];
      if (local_csum == b)
      {
        //printf("%d byte packet\n", expected_len);
        openarms::MinimalSensors msg;

        msg.sensors.resize(4);

        msg.sensors[0] = (pkt[0] << 8) | pkt[1];
        msg.sensors[1] = (pkt[2] << 8) | pkt[3];
        msg.sensors[2] = (pkt[4] << 8) | pkt[5];
        msg.sensors[3] = (pkt[6] << 8) | pkt[7];

        msg.accels.resize(3);

        msg.accels[0] = (pkt[ 8] << 8) | pkt[9];
        msg.accels[1] = (pkt[10] << 8) | pkt[11];
        msg.accels[2] = (pkt[12] << 8) | pkt[13];

        for (uint8_t i = 0; i < msg.accels.size() / 3; i++)
          printf("%6d %6d %6d\n",
                 msg.accels[i*3+0], msg.accels[i*3+1], msg.accels[i*3+2]);
        printf("%6d %6d %6d %6d\n",
               msg.sensors[0], msg.sensors[1], msg.sensors[2], msg.sensors[3]);
        /*
        for (uint8_t i = 0; i < 2; i++)
          printf("%6d %6d %6d %6d\n",
                 msg.sensors[4+i*4+0],
                 msg.sensors[4+i*4+1],
                 msg.sensors[4+i*4+2],
                 msg.sensors[4+i*4+3]);
          */
        printf("\n");
/*        
        for (uint8_t i = 0; i < expected_len; i++)
          printf("%d: %x\n", i, pkt[i]);
        printf("\n\n");
*/
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
#endif

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
  printf("wrote %04d bytes ", (int)sizeof(pkt));
  for (size_t i = 0; i < NUM_MOTORS; i++)
    printf(" %x-%03d ", msg->mode[i], msg->vel[i]);
  printf("\n");
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
        printf("    %d\n", read_buf[i]);
        //if (process_byte(read_buf[i], &pub))
        //  ros::spinOnce();
      }
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

