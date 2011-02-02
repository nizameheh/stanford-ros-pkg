#ifndef ANCIENT_POWERCUBE
#define ANCIENT_POWERCUBE

#include <cmath>
#include "serial_port/lightweightserial.h"

// CREDIT: The vast majority of this code was ported from Player.
// And I think some of this code originated in CARMEN.

class AncientPowercube
{
public:
  AncientPowercube(const char *port, uint32_t baud);
  ~AncientPowercube();

  bool home();
  bool query_pan_tilt();
  bool query_pan();
  float get_pan() { return pan; }
  float get_tilt() { return tilt; }
  bool set_pan_pos(float pan_request);
  bool set_tilt_pos(float tilt_request);

private:
  LightweightSerial *serial;
  bool ready;
  //int ser_fd;
  const static int AMTEC_MAX_CMDSIZE = 48, AMTEC_STX = 0x02, AMTEC_ETX = 0x03;
  const static int AMTEC_DLE = 0x10;
  const static int AMTEC_MODULE_TILT = 11, AMTEC_MODULE_PAN = 12;
  const static int AMTEC_SLEEP_TIME_MS = 20;
  const static int AMTEC_CMD_RESET = 0x00, AMTEC_CMD_HOME = 0x01;
  const static int AMTEC_CMD_HALT = 0x02, AMTEC_CMD_SET_EXT = 0x08;
  const static int AMTEC_CMD_GET_EXT = 0x0a, AMTEC_CMD_SET_MOTION = 0x0b;
  const static int AMTEC_MOTION_FSTEP_ACK = 16, AMTEC_MOTION_FVEL_ACK = 17;
  const static int AMTEC_PARAM_ACT_POS = 0x3c, AMTEC_PARAM_CUBESTATE = 0x27;
  const static int AMTEC_STATE_HOME_OK = 0x02;
  const static float PAN_MOUNT_BIAS;

  float pan, tilt, pan_bias, tilt_bias;
  float last_pan, last_tilt;

  bool send_command(int id, uint8_t *cmd, unsigned len);
  int read_answer(uint8_t *buf, unsigned len);
  int await_answer(uint8_t *buf, unsigned len);
  int await_etx(uint8_t *buf, unsigned len);
  unsigned convert_buffer(uint8_t *buf, unsigned len);
  bool get_float_param(int id, int param, float *val);
  bool get_uint32_param(int id, int param, unsigned *val);
  bool set_pan_vel(float pan_vel_request);
  bool set_tilt_vel(float tilt_vel_request);

  bool reset();

  int timed_serial_read(uint8_t *buf, uint32_t read_len, double max_seconds);
};

#endif
