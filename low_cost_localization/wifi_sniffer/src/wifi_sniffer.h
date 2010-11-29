#ifndef WIFI_SNIFFER_H
#define WIFI_SNIFFER_H

#include "iwlib.h"
#include "ros/time.h"
#include <vector>
#include <string>

class WifiSniff
{
public:
  ros::Time t;
  std::string mac;
  std::string ssid;
  int signal_level, signal_noise;
};

class WifiSniffer
{
public:
  WifiSniffer();
  ~WifiSniffer();

  bool init(const char *iface_cstr);
  bool scan(std::vector<WifiSniff> &sniff);

  std::string iface;
  int sock; 
  iw_range range;
};

#endif

