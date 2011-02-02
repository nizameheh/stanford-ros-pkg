#include "ancient_powercube/ancient_powercube.h"
#include <unistd.h>
#include <cstring>
#include <cstdio>
//#include "serial.h"

const float AncientPowercube::PAN_MOUNT_BIAS = 0; //-30.0*M_PI/180;

AncientPowercube::AncientPowercube(const char *port, uint32_t baud)
: ready(false), last_pan(0), last_tilt(0)
{
  printf("opening serial port [%s] at %d bps\n", port, baud);
  serial = new LightweightSerial(port, baud);
  //if (dgc_serial_connect(&ser_fd, (char *)port, 38400))
  if (!serial->is_ok())
  {
    printf("couldn't open serial port\n");
    return;
  }
  query_pan_tilt();
  home();
  reset();
  usleep(1000000);
  query_pan_tilt();
  pan_bias = pan;
  tilt_bias = tilt;
  ready = true;
  set_pan_pos(0); // go to the pan pos that is defined by the PAN_MOUNT_BIAS
}

AncientPowercube::~AncientPowercube()
{
  home();
  delete serial;
  //if (ser_fd > 0)
  //  dgc_serial_close(ser_fd);
}

bool AncientPowercube::set_pan_vel(float pan_vel_request)
{
  const double MAX_PAN_VEL = 0.5;
  if (pan_vel_request > MAX_PAN_VEL)
    pan_vel_request = MAX_PAN_VEL;
  else if (pan_vel_request < -MAX_PAN_VEL)
    pan_vel_request = -MAX_PAN_VEL;
  uint8_t buf[AMTEC_MAX_CMDSIZE], cmd[6];
  cmd[0] = AMTEC_CMD_SET_MOTION;
  cmd[1] = AMTEC_MOTION_FVEL_ACK;
  memcpy(cmd+2, &pan_vel_request, 4);
  send_command(AMTEC_MODULE_PAN, cmd, 6);
  return read_answer(buf, sizeof(buf)) >= 0;
}

bool AncientPowercube::set_tilt_vel(float tilt_vel_request)
{
  const double MAX_TILT_VEL = 0.5;
  if (tilt_vel_request > MAX_TILT_VEL)
    tilt_vel_request = MAX_TILT_VEL;
  else if (tilt_vel_request < -MAX_TILT_VEL)
    tilt_vel_request = -MAX_TILT_VEL;
  uint8_t buf[AMTEC_MAX_CMDSIZE], cmd[6];
  cmd[0] = AMTEC_CMD_SET_MOTION;
  cmd[1] = AMTEC_MOTION_FVEL_ACK;
  memcpy(cmd+2, &tilt_vel_request, 4);
  send_command(AMTEC_MODULE_TILT, cmd, 6);
  return read_answer(buf, sizeof(buf)) >= 0;
}

bool AncientPowercube::set_pan_pos(float pan_request)
{
  const float PAN_VEL = 0.5; // * M_PI / 180;
  uint8_t buf[AMTEC_MAX_CMDSIZE], cmd[8];
  unsigned short time;
  pan_request += PAN_MOUNT_BIAS;
  time = (unsigned short)rint((fabs(pan_request - last_pan) / PAN_VEL) * 1000);
  cmd[0] = AMTEC_CMD_SET_MOTION;
  cmd[1] = AMTEC_MOTION_FSTEP_ACK;
  memcpy(cmd+2, &pan_request, 4);
  memcpy(cmd+6, &time, 2);
  send_command(AMTEC_MODULE_PAN, cmd, 8);
  if (read_answer(buf, sizeof(buf)) < 0)
    return false;
  last_pan = pan_request;
  return true;
}

bool AncientPowercube::set_tilt_pos(float tilt_req)
{
  const float TILT_VEL = 0.5; // * M_PI / 180;
  uint8_t buf[AMTEC_MAX_CMDSIZE], cmd[8];
  unsigned short time;
  time = (unsigned short)rint((fabs(tilt_req - last_tilt) / TILT_VEL) * 1000);
  cmd[0] = AMTEC_CMD_SET_MOTION;
  cmd[1] = AMTEC_MOTION_FSTEP_ACK;
  memcpy(cmd+2, &tilt_req, 4);
  memcpy(cmd+6, &time, 2);
  send_command(AMTEC_MODULE_TILT, cmd, 8);
  if (read_answer(buf, sizeof(buf)) < 0)
    return false;
  last_tilt = tilt_req;
  return true;
}

bool AncientPowercube::home()
{
  uint8_t buf[AMTEC_MAX_CMDSIZE], cmd[1];
  unsigned state;
  cmd[0] = AMTEC_CMD_HOME;
  send_command(AMTEC_MODULE_PAN, cmd, 1);
  if (read_answer(buf, sizeof(buf)) < 0)
    return false;
  int attempt = 0;
  const int MAX_ATTEMPTS = 500;
  for (attempt = 0; attempt < MAX_ATTEMPTS; attempt++)
  {
    usleep(AMTEC_SLEEP_TIME_MS);
    if (!get_uint32_param(AMTEC_MODULE_PAN, AMTEC_PARAM_CUBESTATE, &state))
      return false;
    if (state & AMTEC_STATE_HOME_OK)
      break;
  }
  if (attempt == MAX_ATTEMPTS)
  {
    printf("pan motor never went home. how sad\n");
    return false;
  }
  send_command(AMTEC_MODULE_TILT, cmd, 1);
  if (read_answer(buf, sizeof(buf)) < 0)
    return false;
  for (attempt = 0; attempt < MAX_ATTEMPTS; attempt++)
  {
    usleep(AMTEC_SLEEP_TIME_MS);
    if (!get_uint32_param(AMTEC_MODULE_TILT, AMTEC_PARAM_CUBESTATE, &state))
      return false;
    if (state & AMTEC_STATE_HOME_OK)
      break;
  }
  if (attempt == MAX_ATTEMPTS)
  {
    printf("tilt motor never went home. how sad\n");
    return false;
  }
  //printf("home OK\n");
  return true;	
}

bool AncientPowercube::query_pan()
{
  float tmp;
  if (!get_float_param(AMTEC_MODULE_PAN, AMTEC_PARAM_ACT_POS, &tmp))
    return false;
  pan = tmp - PAN_MOUNT_BIAS;
  return true;
}

bool AncientPowercube::query_pan_tilt()
{
  float tmp;
  if (!get_float_param(AMTEC_MODULE_PAN, AMTEC_PARAM_ACT_POS, &tmp))
    return false;
  pan = tmp;
  if (!get_float_param(AMTEC_MODULE_TILT, AMTEC_PARAM_ACT_POS, &tmp)) 
    return false;
  tilt = tmp;
  //printf("pan,tilt = %f, %f\n", (pan-pan_bias)*180/M_PI, (tilt-tilt_bias)*180/M_PI);
  return true;
}

bool AncientPowercube::get_float_param(int id, int param, float *val)
{
  uint8_t buf[AMTEC_MAX_CMDSIZE], cmd[2];
  cmd[0] = AMTEC_CMD_GET_EXT;
  cmd[1] = param;
  send_command(id, cmd, 2);
  if (read_answer(buf, sizeof(buf)) < 0)
    return false;
  *val = *((float *)(buf+4));
  return true;
}

bool AncientPowercube::get_uint32_param(int id, int param, unsigned *val)
{
  uint8_t buf[AMTEC_MAX_CMDSIZE], cmd[2];
  cmd[0] = AMTEC_CMD_GET_EXT;
  cmd[1] = param;
  send_command(id, cmd, 2);
  if (read_answer(buf, sizeof(buf)) < 0)
    return false;
  *val = *((unsigned *)(buf+4));
  return true;
}

unsigned AncientPowercube::convert_buffer(uint8_t *buf, unsigned len)
{
  unsigned i, j, actual_len = len;
  actual_len = len;
  for (i=0;i<len;i++)
  {
    if(buf[i] == AMTEC_DLE)
    {
      switch(buf[i+1])
      {
        case 0x82:
          buf[i] = 0x02;
          for (j=i+2; j<len; j++)
            buf[j-1] = buf[j];
          actual_len--;
          break;
        case 0x83:
          buf[i] = 0x03;
          for(j = i + 2; j < len; j++)
            buf[j-1] = buf[j];
          actual_len--;
          break;
        case 0x90:
          buf[i] = 0x10;
          for(j=i+2;j<len;j++)
            buf[j-1] = buf[j];
          actual_len--;
          break;
      }
    }
  }
  return actual_len;
}

int AncientPowercube::timed_serial_read(uint8_t *buf, uint32_t read_len,
                                        double max_seconds)
{
  //return dgc_serial_readn(ser_fd, buf, read_len, max_seconds);
  uint8_t *write_ptr = buf;
  uint32_t total_read = 0;
  for (int attempt = 0; attempt < (int)(max_seconds / 0.01) + 1; attempt++)
  {
    if (!serial->is_ok())
    {
      printf("serial in bad shape\n");
      return false;
    }
    int32_t nread = serial->read_block(write_ptr, read_len - total_read);
    if (nread < 0)
      break;
    total_read += nread;
    write_ptr += nread;
    if (total_read >= read_len)
      break;
    usleep(10000); // wait 10ms and see if there is more data
  }
  return total_read;
}

int AncientPowercube::await_etx(uint8_t *buf, unsigned len)
{
  //printf("await_etx\n");
  int pos = 0, loop = 0, numread, totalnumread;
  while (loop < 10)
  {
    if ((numread = timed_serial_read(buf+pos, len-pos, 0.1)) < 0)
    //if ((numread = dgc_serial_readn(ser_fd, buf+pos, len-pos, 0.1)) < 0)
    {
      printf("read error\n");
      return -1;
    }
    else if (numread == 0)
    {
      loop++;
    }
    else
    {
      //printf("numread = %d\n", numread);
      if (buf[pos+numread-1] == AMTEC_ETX)
      {
        totalnumread = pos+numread-1;
        return totalnumread;
      }
      pos += numread;
    }
  }
  printf("never found etx\n");
  return -1;
}

int AncientPowercube::await_answer(uint8_t *buf, unsigned len)
{
  const int MAX_ATTEMPTS = 4;
  for (int attempt = 0; attempt < MAX_ATTEMPTS; attempt++)
  {
    if (timed_serial_read(buf, 1, 0.5) > 0)
    //if (dgc_serial_readn(ser_fd, &buf[0], 1, 0.5))
    {
      if (buf[0] == AMTEC_STX)
        return await_etx(buf, len);			
    }
  }
  return -1; // never got an answer
}

int AncientPowercube::read_answer(uint8_t *buf, unsigned len)
{
  int act_len;
  if ((act_len = await_answer(buf, len)) <= 0)
    return act_len;
  else
    return convert_buffer(buf, act_len);
}

bool AncientPowercube::reset()
{
  uint8_t buf[AMTEC_MAX_CMDSIZE], cmd[1];
  cmd[0] = AMTEC_CMD_RESET;
  send_command(AMTEC_MODULE_PAN, cmd, 1);
  if (read_answer(buf, sizeof(buf)) < 0)
  {
    printf("readanswer() failed\n");
    return false;
  }
  if (!send_command(AMTEC_MODULE_TILT, cmd, 1))
  {
    printf("send command() failed in reset\n");
    return false;
  }
  if (read_answer(buf, sizeof(buf)) < 0)
  {
    printf("readanswer() failed\n");
    return false;
  }
  //printf("reset OK\n");
  return true;
}

bool AncientPowercube::send_command(int id, uint8_t *cmd, unsigned len)
{
  unsigned i;
  int ctr, add = 0;
  unsigned char rcmd[AMTEC_MAX_CMDSIZE], bcc; 
  unsigned char lmnr = (id & 7) << 5;
  unsigned char umnr = (id >> 3) | 4;

  for (i=0;i<len;i++) 
  {
    if ( (cmd[i]==0x02) ||
        (cmd[i]==0x03) ||
        (cmd[i]==0x10) )
      add++;
  }

  lmnr = lmnr + len;
  rcmd[0] = AMTEC_STX;
  rcmd[1] = umnr;
  rcmd[2] = lmnr;
  ctr = 3;

  for (i=0;i<len;i++) 
  {
    switch(cmd[i]) 
    {
      case 0x02:
        rcmd[ctr] = 0x10;
        rcmd[++ctr] = 0x82;
        break;
      case 0x03:
        rcmd[ctr] = 0x10;
        rcmd[++ctr] = 0x83;
        break;
      case 0x10:
        rcmd[ctr] = 0x10;
        rcmd[++ctr] = 0x90;
        break;
      default:
        rcmd[ctr] = cmd[i];
    }
    ctr++;
  }
  bcc = id;
  for (i=0;i<len;i++)
    bcc += cmd[i];
  bcc = bcc + (bcc>>8);
  switch(bcc) 
  {
    case 0x02:
      rcmd[ctr++] = 0x10;
      rcmd[ctr++] = 0x82;
      break;
    case 0x03:
      rcmd[ctr++] = 0x10;
      rcmd[ctr++] = 0x83;
      break;
    case 0x10:
      rcmd[ctr++] = 0x10;
      rcmd[ctr++] = 0x90;
      break;
    default:
      rcmd[ctr++] = bcc;
  }
  rcmd[ctr++] = AMTEC_ETX;
/*
  printf("writing  ");
  for (int i = 0; i < ctr; i++)
    printf("%02x ", rcmd[i]);
  printf("\n");
*/
  if (!serial->write_block(rcmd, ctr))
    return false;
  usleep(1000);
  return true;
}

