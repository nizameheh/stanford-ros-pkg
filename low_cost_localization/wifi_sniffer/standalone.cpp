#include "wifi_sniffer.h"
#include "ros/time.h"
#include "iwlib.h"
#include <cstdio>
#include <cstdlib>
#include <signal.h>
using std::vector;

static bool g_done = false;
void sigint_handler(int sig) { g_done = true; }

int main(int argc, char **argv)
{
  ros::Time::init();
  if (argc != 2)
  {
    fprintf(stderr, "usage: standalone DEVICE\n");
    return 1;
  }
  WifiSniffer sniffer;
  if (!sniffer.init(argv[1]))
    return 1;
  signal(SIGINT, sigint_handler);
  vector<WifiSniff> sniffs;
  while (!g_done)
  {
    sniffer.scan(sniffs);
    printf("\n");
    for (vector<WifiSniff>::iterator it = sniffs.begin();
         it != sniffs.end(); ++it)
      printf("%d %s %s\n", it->signal_level, it->mac.c_str(), it->ssid.c_str());
    usleep(100000);
  }
  return 0;
}

