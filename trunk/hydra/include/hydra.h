#ifndef HYDRA_H
#define HYDRA_H

#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <linux/hidraw.h>
#include <usb.h>
#include <LinearMath/btQuaternion.h>
#include <LinearMath/btVector3.h>

namespace hydra
{

class Hydra
{
public:
  Hydra();
  ~Hydra();
  int hidraw_fd;

  bool init(const char *device);
  bool poll(uint32_t ms_to_wait);

  int16_t raw_pos[6], raw_quat[8];
  uint8_t raw_buttons[2];
  int16_t raw_analog[6];

  btVector3 pos[2];
  btQuaternion quat[2];
  float analog[6];
  uint8_t buttons[14];
};

}

#endif

