#include <cstdlib>
#include <cstdio>
#include "ros/ros.h"
#include "ancient_powercube/GotoPanTilt.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ancient_powercube_client");
  if (argc != 3)
  {
    ROS_INFO("usage: set_pan_tilt PAN TILT\n");
    return 1;
  }
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<ancient_powercube::GotoPanTilt>("goto_pan_tilt");
  ancient_powercube::GotoPanTilt srv_call;
  srv_call.request.pan = atof(argv[1]);
  srv_call.request.tilt = atof(argv[2]);
  if (client.call(srv_call))
    ROS_INFO("awesum");
  else
    ROS_ERROR("bogus");
  return 0;
}
