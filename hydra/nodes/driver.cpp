#include <string>
#include <ros/ros.h>
#include "hydra.h"
#include "hydra/Raw.h"
#include "hydra/Calib.h"
#include "tf/tf.h"
using namespace hydra;
using std::string;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hydra_driver");
  ros::NodeHandle n, n_private("~");
  ros::Publisher raw_pub = n.advertise<hydra::Raw>("hydra_raw", 1);
  ros::Publisher calib_pub = n.advertise<hydra::Calib>("hydra_calib", 1);
  Hydra h;
  string dev;
  n_private.param<std::string>("device", dev, "/dev/hidraw0");
  ROS_INFO("opening hydra on %s", dev.c_str());
  if (!h.init(dev.c_str()))
  {
    ROS_FATAL("couldn't open hydra on %s", dev.c_str());
    return 1;
  }
  ROS_INFO("starting stream...");
  while (n.ok())
  {
    if (h.poll(10))
    {
      hydra::Raw msg;
      msg.header.stamp = ros::Time::now();
      for (int i = 0; i < 6; i++)
        msg.pos[i] = h.raw_pos[i];
      for (int i = 0; i < 8; i++)
        msg.quat[i] = h.raw_quat[i];
      for (int i = 0; i < 2; i++)
        msg.buttons[i] = h.raw_buttons[i];
      for (int i = 0; i < 6; i++)
        msg.analog[i] = h.raw_analog[i];
      raw_pub.publish(msg);

      hydra::Calib c_msg;
      c_msg.header.stamp = msg.header.stamp;
      for (int i = 0; i < 2; i++)
        tf::transformTFToMsg(btTransform(h.quat[i], h.pos[i]),
                             c_msg.paddles[i].transform);
      for (int i = 0; i < 7; i++)
      {
        c_msg.paddles[0].buttons[i] = h.buttons[i];
        c_msg.paddles[1].buttons[i] = h.buttons[i+7];
      }
      for (int i = 0; i < 2; i++)
      {
        c_msg.paddles[0].joy[i] = h.analog[i];
        c_msg.paddles[1].joy[i] = h.analog[i+3];
      }
      c_msg.paddles[0].trigger = h.analog[2];
      c_msg.paddles[1].trigger = h.analog[5];
      calib_pub.publish(c_msg);
      ros::spinOnce();
    }
  }
  return 0;
}

