#include "ros/console.h"
#include "wifi_sniffer.h"
using std::string;

WifiSniffer::WifiSniffer()
: sock(-1)
{
}

WifiSniffer::~WifiSniffer()
{
  if (sock >= 0)
    iw_sockets_close(sock);
}

bool WifiSniffer::init(const char *iface_cstr)
{
  iface = string(iface_cstr);
  sock = iw_sockets_open();
  if (sock < 0)
  {
    ROS_ERROR("couldn't open network socket on %s", iface.c_str());
    return false;
  }
  if (iw_get_range_info(sock, iface.c_str(), &range) < 0)
  {
    ROS_ERROR("interface %s doesn't support scanning\n", iface.c_str());
    return false;
  }
  return true;
}

bool WifiSniffer::scan(std::vector<WifiSniff> &sniffs)
{
  sniffs.clear();
  iwreq wrq;
  uint8_t *scan_buffer = NULL;
  int scan_buflen = IW_SCAN_MAX_DATA;
  timeval tv;
  int timeout = 10000000; // bail if it takes longer than 10 seconds

  tv.tv_sec = 0;
  tv.tv_usec = 250000;

  wrq.u.data.pointer = NULL;
  wrq.u.data.length = 0;
  wrq.u.data.length = 0;

  // initiate scanning
  if (iw_set_ext(sock, iface.c_str(), SIOCSIWSCAN, &wrq) < 0)
  {
    if (errno == EBUSY)
      ROS_ERROR("device busy");
    else if (errno == EPERM)
      ROS_ERROR("couldn't start a scan. insufficient permissions?");
    else 
      ROS_ERROR("interface %s doesn't support scanning?: %s",
                iface.c_str(), strerror(errno));
    return false;
  }
  timeout -= tv.tv_usec;
  for (bool scan_complete = false; !scan_complete; )
  {
    fd_set fds;
    int last_fd;
    int ret;
    FD_ZERO(&fds);
    last_fd = -1;
    ret = select(last_fd+1, &fds, NULL, NULL, &tv);
    if (ret < 0)
    {
      if (errno == EAGAIN || errno == EINTR)
        continue;
      ROS_ERROR("unhandled signal... bye");
      return false;
    }
    if (ret == 0)
    {
      uint8_t *scan_newbuf;
      while (!scan_complete)
      {
        scan_newbuf = (uint8_t *)realloc(scan_buffer, scan_buflen);
        if (scan_newbuf == NULL)
        {
          if (scan_buffer)
            free(scan_buffer);
          ROS_ERROR("malloc fail. system over.");
          return false;
        }
        scan_buffer = scan_newbuf;
        wrq.u.data.pointer = scan_buffer;
        wrq.u.data.flags = IW_SCAN_ALL_ESSID | IW_SCAN_ALL_FREQ | IW_SCAN_ALL_MODE | IW_SCAN_ALL_RATE;
        wrq.u.data.length = scan_buflen;
        if (iw_get_ext(sock, iface.c_str(), SIOCGIWSCAN, &wrq) < 0)
        {
          if (errno == E2BIG && range.we_version_compiled > 16)
          {
            if (wrq.u.data.length > scan_buflen)
              scan_buflen = wrq.u.data.length;
            else
              scan_buflen *= 2; // grow buffer
            //printf("grow buffer\n");
            continue;
          }
          else if (errno == EAGAIN)
          {
            tv.tv_sec = 0;
            tv.tv_usec = 250000;
            timeout -= tv.tv_usec;
            if (timeout < 0)
            {
              scan_complete = true;
              ROS_ERROR("hit scan timeout");
            }
            break;
          }
          // if we get here, something bad has happened
          free(scan_buffer);
          ROS_ERROR("failed to read scan data: %s", strerror(errno));
          return false;
        }
        else
          scan_complete = true;
      }
    }
  }

  if (wrq.u.data.length)
  {
    ros::Time scan_end_time = ros::Time::now();
    WifiSniff sniff;
    sniff.t = scan_end_time;
    iw_event iwe;
    stream_descr stream;
    iw_init_event_stream(&stream, (char *)scan_buffer, wrq.u.data.length);
    int ret = 0;
    do
    {
      ret = iw_extract_event_stream(&stream, &iwe, range.we_version_compiled);
      if (ret > 0)
      {
        switch (iwe.cmd)
        {
          case SIOCGIWAP:
          {
            //printf("\naddress %s  ",
            //       iw_saether_ntop(&iwe.u.ap_addr, (char *)scan_buffer));
            char bufp[1024]; // just don't run over the end plz
            iw_ether_ntop((const struct ether_addr *)&iwe.u.ap_addr, bufp);
            sniff.mac = string(bufp);
            break;
          }
          case IWEVQUAL:
            if (iwe.u.qual.level != 0 || 
                (iwe.u.qual.updated & (IW_QUAL_DBM | IW_QUAL_RCPI)))
            {
              sniff.signal_level = iwe.u.qual.level;
              sniff.signal_noise = iwe.u.qual.noise;
            }
            break;
#if 0 
          case IWEVCUSTOM:
          {
            if (iwe.u.data.pointer && iwe.u.data.length)
            {
              char custom_text[IW_CUSTOM_MAX+1];
              memcpy(custom_text, iwe.u.data.pointer, iwe.u.data.length);
              custom_text[iwe.u.data.length] = 0; // null terminate plz
              printf("extra: [%s]\n", custom_text);
              break;
            }
          }
#endif
          case SIOCGIWESSID:
          {
            char essid[IW_ESSID_MAX_SIZE+1];
            memset(essid, 0, sizeof(essid));
            if (iwe.u.essid.pointer && iwe.u.essid.length)
              memcpy(essid, iwe.u.essid.pointer, iwe.u.essid.length);
            if (iwe.u.essid.flags)
              sniff.ssid = string(essid);
            else
              sniff.ssid = string("");
            sniffs.push_back(sniff); // this event seems to always happen last
                                     // I'm assuming it always does! i hope 
                                     // this isn't just due to the wifi driver 
                                     // i'm using or something odd like that
            break;
          }
        }
      }
    } while (ret > 0);
    ROS_INFO("sniffed %d access points", (int)sniffs.size());
  }
  else
    ROS_INFO("no scan results");

  free(scan_buffer);
  return true;
}

