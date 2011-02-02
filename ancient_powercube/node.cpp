#include <cstdlib>
#include <cstdio>
#include "ancient_powercube/ancient_powercube.h"
#include "ancient_powercube/GotoPanTilt.h"
#include "ros/ros.h"
#include <string>
using std::string;

AncientPowercube *g_pc = NULL;

bool goto_service(ancient_powercube::GotoPanTilt::Request &req,
                  ancient_powercube::GotoPanTilt::Response &res)
{
  ROS_INFO("goto request: %.2f, %.2f", req.pan, req.tilt);
  g_pc->set_pan_pos(req.pan);
  g_pc->set_tilt_pos(req.tilt);
  res.ok = true;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ancient_powercube");
  ros::NodeHandle n, n_private("~");
  string port;
  int baud;
  n_private.param<std::string>("port", port, "/dev/ttyUSB0");
  n_private.param("baud", baud, 38400);

  AncientPowercube pc(port.c_str(), baud);
  g_pc = &pc;
  
  //ros::Publisher state_pub = n.advertise<ancient_powercube::Status>("pantilt_state", 100);
  ros::ServiceServer service = n.advertiseService("goto_pan_tilt", goto_service);

  while (n.ok())
  {
    if (!pc.query_pan_tilt())
    {
      ROS_FATAL("lost contact with the powercube. bai.\n");
      break;
    }
    /*
    ancient_powercube::Status status;
    status.pan = pc.get_pan();
    status.tilt = pc.get_tilt();
    status.header.stamp = ros::Time::now();
    state_pub.publish(status);
    */
    ros::spinOnce();
    usleep(1000);
  }

  return 0;
}
