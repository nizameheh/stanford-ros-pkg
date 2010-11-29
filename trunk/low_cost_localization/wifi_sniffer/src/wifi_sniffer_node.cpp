#include "wifi_sniffer.h"
#include "wifi_sniffer/WifiSniff.h"
#include "wifi_sniffer/WifiScan.h"
#include <ros/ros.h>
#include <vector>
using std::vector;
using std::string;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wifi_sniffer_node");
  ros::NodeHandle n, n_private("~");
  ros::Publisher wifi_scan_pub = n.advertise<wifi_sniffer::WifiScan>("wifi_scan", 100);
  WifiSniffer sniffer;
  string iface;
  n_private.param<string>("iface", iface, "wlan0");

  if (!sniffer.init(iface.c_str()))
  {
    ROS_FATAL("ahhh couldn't start sniffing on %s", iface.c_str());
    return 1;
  }
  vector<WifiSniff> sniffs;

  while (ros::ok())
  {
    if (!sniffer.scan(sniffs))
      ROS_ERROR("couldn't scan!");
    else
    {
      wifi_sniffer::WifiScan scan_msg;
      scan_msg.t = ros::Time::now();
      for (vector<WifiSniff>::iterator it = sniffs.begin();
           it != sniffs.end(); ++it)
      {
        wifi_sniffer::WifiSniff sniff_msg;
        sniff_msg.t = it->t;
        sniff_msg.mac = it->mac;
        sniff_msg.ssid = it->ssid;
        sniff_msg.signal_level = it->signal_level;
        sniff_msg.signal_noise = it->signal_noise;
        scan_msg.sniffs.push_back(sniff_msg);
      }
      wifi_scan_pub.publish(scan_msg);
    }
    ros::spinOnce();
    ros::Duration(0.1).sleep(); // some drivers need to have some wait before
                                // they scan again, or else they just give the
                                // cached results...
  }
  return 0;
}
