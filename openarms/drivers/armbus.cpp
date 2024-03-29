// copyright 2009 Morgan Quigley, mquigley@cs.stanford.edu
// bsd license blah blah

#include <unistd.h>
#include <cstdio>
#include <ctime>
#include <cstdlib>
#include <string>
#include <csignal>
#include "serial_port/lightweightserial.h"
#include "ros/time.h"
#include "ros/ros.h"
#include "openarms/ArmActuators.h"
#include "openarms/ArmSensors.h"

LightweightSerial *g_serial = NULL; 
enum rx_state_t { PING, STEPPER_POS_0, STEPPER_POS_1, STEPPER_ACCEL_1, 
                  SERVO_POS_0, SERVO_POS_1, SERVO_POS_2, SERVO_POS_3,
                  ELBOW_ENCODER,
                  STEPPER_ENCODER_0, STEPPER_ENCODER_1 } g_rx_state;

uint16_t g_stepper_timers[4];
uint16_t g_servo_torques[4], g_servo_dirs[4];
// the servos are assumed to be programmed as 0,1,..,NUM_MOTORS
openarms::ArmSensors sensors_msg;
int g_run_motors = 1;

bool process_byte(uint8_t b, ros::Publisher *pub)
{
  static uint8_t pkt[256], servo_id = 0, s_err = 0;
  static uint8_t write_pos = 0, expected_len = 0;
  static enum { PREAMBLE1, PREAMBLE2, ID, LEN, ERR, DATA, CHECKSUM } s_state;
  switch (s_state)
  {
    case PREAMBLE1:
      if (b == 0xff)
        s_state = PREAMBLE2;
      else
        printf("%x found but 0xff expected\n", b);
      break;
    case PREAMBLE2:
      if (b == 0xff)
        s_state = ID;
      else
      {
        printf("%x found but 0xff expected\n", b);
        s_state = PREAMBLE1; // try to re-sync
      }
      break;
    case ID:
      servo_id = b;
      s_state = LEN;
      break;
    case LEN:
      expected_len = b;
      if (expected_len > 20)
      {
        s_state = PREAMBLE1; // fail. try to re-sync.
        printf("expected a length, but got %x\n", b);
      }
      else
        s_state = ERR;
      write_pos = 0;
      break;
    case ERR:
      s_err = b;
      if (!s_err)
      {
        if (expected_len > 2)
          s_state = DATA;
        else
          s_state = CHECKSUM;
      }
      else
        s_state = CHECKSUM;
      break;
    case DATA:
      pkt[write_pos++] = b;
      if (write_pos >= expected_len-2)
        s_state = CHECKSUM;
      break;
    case CHECKSUM:
      s_state = PREAMBLE1;
      uint8_t local_csum = servo_id + expected_len + s_err;
      //printf("id = %d  len = %d  err = %x\n", servo_id, expected_len, s_err);
      for (int i = 0; !s_err && i < expected_len-2; i++)
        local_csum += pkt[i];
      local_csum = ~local_csum;
      if (local_csum == b)
      {
        //printf("packet from %d\n", servo_id);
        if ((g_rx_state == STEPPER_POS_0 || g_rx_state == STEPPER_POS_1)
            && expected_len == 8+2)
        {
          uint32_t pos_0 = (uint32_t) pkt[0]        + 
                           (uint32_t)(pkt[1] << 8)  +
                           (uint32_t)(pkt[2] << 16) +
                           (uint32_t)(pkt[3] << 24);
          uint32_t pos_1 = (uint32_t) pkt[4]        + 
                           (uint32_t)(pkt[5] << 8)  +
                           (uint32_t)(pkt[6] << 16) +
                           (uint32_t)(pkt[7] << 24);
          //printf("%d: %10u %10u\n", servo_id, pos_0, pos_1);
          if (g_rx_state == STEPPER_POS_0)
          {
            sensors_msg.pos[0] =  pos_0;
            sensors_msg.pos[1] = -pos_1;
            pub->publish(sensors_msg);
          }
          else
          {
            sensors_msg.pos[2] = pos_0;
            sensors_msg.pos[3] = pos_1;
            //printf("%d: %10u %10u\n", servo_id, pos_0, pos_1);
          }
          return true;
        }
        else if (g_rx_state == SERVO_POS_0 ||
                 g_rx_state == SERVO_POS_1 ||
                 g_rx_state == SERVO_POS_2 ||
                 g_rx_state == SERVO_POS_3 )
        {
          uint16_t pos = pkt[0] + (uint16_t)(pkt[1] << 8);
          if (g_rx_state == SERVO_POS_0)
            sensors_msg.pos[4] = pos;
          else if (g_rx_state == SERVO_POS_1)
            sensors_msg.pos[5] = pos;
          else if (g_rx_state == SERVO_POS_2)
            sensors_msg.pos[6] = pos;
          else if (g_rx_state == SERVO_POS_3)
            sensors_msg.pos[7] = pos;
          //printf("%d: %5d\n", servo_id, pos);
          return true;
        }
        else if (g_rx_state == STEPPER_ENCODER_0)
        {
          //uint16_t encoder_0 = (uint32_t) pkt[0]        + 
          //                     (uint32_t)(pkt[1] << 8);
          uint16_t encoder_1 = (uint32_t) pkt[2]        + 
                               (uint32_t)(pkt[3] << 8);
          //printf("encoders 10: %06d  %06d\n", encoder_0, encoder_1);
          sensors_msg.encoder[0] = encoder_1;
          return true;
        }
        else if (g_rx_state == STEPPER_ENCODER_1)
        {
          uint16_t encoder_0 = (uint32_t) pkt[0]        + 
                               (uint32_t)(pkt[1] << 8);
          uint16_t encoder_1 = (uint32_t) pkt[2]        + 
                               (uint32_t)(pkt[3] << 8);
          //printf("encoders 11: %06d  %06d\n", encoder_0, encoder_1);
          sensors_msg.encoder[1] = encoder_0;
          sensors_msg.encoder[2] = encoder_1;
          return true;
        }
        else if (g_rx_state == ELBOW_ENCODER)
        {
          uint16_t encoder = (uint32_t) pkt[0]        + 
                             (uint32_t)(pkt[1] << 8);
          sensors_msg.encoder[3] = encoder;
          return true;
        }

        else if (g_rx_state == STEPPER_ACCEL_1)
        {
          return true;
/*
          for (int i = 0; i < 8; i++)
            printf("  %x ", pkt[i]);
          printf("\n");
*/
        }
        else if (g_rx_state == PING)
        {
        }
        /*
        //printf("packet ok\n");
        //printf("%d byte packet\n", expected_len);
        //pub->publish(msg);
        */
        return false; // no need to transmit
      }
      else
        printf("checksum fail: local sum=%x wire sum=%x\n", local_csum, b);
      return false; // sent a message
      break;
  }
  return false; // didn't send a message
}

uint8_t calc_checksum(uint8_t *pkt, uint8_t pkt_len)
{
  uint8_t checksum = 0;
  for (int j = 2; j < pkt_len; j++)
    checksum += pkt[j];
  return ~checksum;
}

void write_data(uint8_t id, uint8_t addr, uint8_t length, uint8_t *data)
{
  uint8_t pkt[300];
  pkt[0] = 0xff;
  pkt[1] = 0xff;
  pkt[2] = id;
  pkt[3] = length + 3;
  pkt[4] = 0x03; // "write data"
  pkt[5] = addr;
  for (int i = 0; i < length; i++)
    pkt[6+i] = data[i];
  pkt[6+length] = calc_checksum(pkt, 6+length);
  g_serial->write_block(pkt, 7+length);
}

void send_torque(uint8_t id, uint16_t torque, uint8_t dir)
{
  if (!g_run_motors)
    return; // SEE YA
  // implemented from the robotis user's manual, page 16
  uint8_t data[20];
  data[0] = (uint8_t)(torque & 0xff);
  data[1] = (uint8_t)((0x3 & (torque >> 8))) | (dir ? 0x04 : 0x00);
  write_data(id, 0x20, 2, data); // "moving speed" register
}

void send_stepper_vel(uint8_t id, uint16_t vel_0, uint16_t vel_1)
{
  if (!g_run_motors)
    return; // adios
  uint8_t data[20];
  data[0] = (uint8_t)(vel_0 & 0xff);
  data[1] = (uint8_t)(vel_0 >> 8);
  data[2] = (uint8_t)(vel_1 & 0xff);
  data[3] = (uint8_t)(vel_1 >> 8);
  write_data(id, 0x20, 4, data);
}

void send_joint_limit(uint8_t id, uint16_t cw, uint16_t ccw)
{
  uint8_t data[20];
  data[0] = (uint8_t)(cw & 0xff);
  data[1] = (uint8_t)((0xff & (cw >> 8)));
  data[2] = (uint8_t)(ccw & 0xff);
  data[3] = (uint8_t)((0xff & (ccw >> 8)));
  write_data(id, 0x06, 4, data); // CW and CCW limit registers (0,0) = wheel
}

void enable_motors(uint8_t enable)
{
  uint8_t data = (enable ? 1 : 0);
  write_data(0xfe, 0x18, 1, &data);
}

void enable_motor(uint8_t id, uint8_t enable)
{
  uint8_t data[2];
  for (int i = 0; i < 2;i ++)
    data[i]  = (enable ? 1 : 0);
  write_data(id, 0x18, 2, data);
}

void enable_led(uint8_t id, uint8_t enable)
{
  uint8_t data = (enable ? 1 : 0);
  //printf("set led on motor %x to %d\n", id, data);
  write_data(id, 0x19, 1, &data);
}

void query_motor(uint8_t id)
{
  uint8_t pkt[20];
  pkt[0] = 0xff;
  pkt[1] = 0xff;
  pkt[2] = id;
  pkt[3] = 4; // 2 parameters + 2
  pkt[4] = 0x02; // "read data"
  pkt[5] = 0x24; // present position
  pkt[6] = 2; // read LSB and MSB
  pkt[7] = calc_checksum(pkt, 7);
  if (id == 1)
    g_rx_state = SERVO_POS_0;
  else if (id == 2)
    g_rx_state = SERVO_POS_1;
  else if (id == 3)
    g_rx_state = SERVO_POS_2;
  else if (id == 4)
    g_rx_state = SERVO_POS_3;
  else
  {
    ROS_INFO("woah there. servo id must be in {1,2,3,4}");
    return;
  }
  g_serial->write_block(pkt, 8);
}

void query_encoder(uint8_t id, uint8_t encoder_count = 1)
{
  uint8_t pkt[20];
  pkt[0] = 0xff;
  pkt[1] = 0xff;
  pkt[2] = id;
  pkt[3] = 4; // 2 parameters + 2
  pkt[4] = 0x02; // "read data"
  pkt[5] = 0x25; // present position + 1 = encoder in my hack of the protocol
  pkt[6] = (encoder_count == 1 ? 2 : 4); // 16-bit or 32-bit word
  pkt[7] = calc_checksum(pkt, 7);
  if (id == 10)
    g_rx_state = STEPPER_ENCODER_0;
  else if (id == 11)
    g_rx_state = STEPPER_ENCODER_1;
  else if (id == 12)
    g_rx_state = ELBOW_ENCODER;
  else
  {
    ROS_INFO("woah there. encoder id must be in {12}");
    return;
  }
  g_serial->write_block(pkt, 8);
}

void query_stepper_positions(uint8_t id)
{
  uint8_t pkt[20];
  pkt[0] = 0xff;
  pkt[1] = 0xff;
  pkt[2] = id;
  pkt[3] = 4; // 2 parameters + 2
  pkt[4] = 0x02; // "read data"
  pkt[5] = 0x24; // present position
  pkt[6] = 8; // 32 bits from both steppers
  pkt[7] = calc_checksum(pkt, 7);
  if (id == 10)
    g_rx_state = STEPPER_POS_0;
  else if (id == 11)
    g_rx_state = STEPPER_POS_1;
  else
  {
    ROS_INFO("woah. stepper id must be 11 or 10");
    return;
  }
  g_serial->write_block(pkt, 8);
}

void query_accelerometer(uint8_t id)
{
  uint8_t pkt[20];
  pkt[0] = 0xff;
  pkt[1] = 0xff;
  pkt[2] = id;
  pkt[3] = 4; // 2 parameters + 2
  pkt[4] = 0x02; // "read data"
  pkt[5] = 0x2b; // "temperature" aka accelerometer
  pkt[6] = 8; // 8 bytes for now (version, X, Y, Z, temperature)
  pkt[7] = calc_checksum(pkt, 7);
  if (id != 11)
  {
    ROS_INFO("woah. only stepper id 11 has an accelerometer");
    return;
  }
  g_rx_state = STEPPER_ACCEL_1;
  g_serial->write_block(pkt, 8);
}

void ping_motor(uint8_t id)
{
  uint8_t pkt[20];
  pkt[0] = 0xff;
  pkt[1] = 0xff;
  pkt[2] = id;
  pkt[3] = 2; // 0 parameters + 2 overhead
  pkt[4] = 0x01; // ping instruction
  pkt[5] = calc_checksum(pkt, 5);
  printf("pinging motor %d, checksum %x\n", id, pkt[5]);
  g_rx_state = PING;
  g_serial->write_block(pkt, 6);
}

void actuators_cb(const openarms::ArmActuators::ConstPtr &msg)
{
  if (msg->stepper_vel.size() != 4)
  {
    ROS_INFO("ahhh was expecting 4 stepper velocities");
    return;
  }
  if (msg->servo_torque.size() != 4)
  {
    ROS_INFO("ahhh was expecting 4 servo torques");
    return;
  }
  // flip joints as needed
  openarms::ArmActuators act = *msg;
  act.stepper_vel[0] *= -1;
  //act.stepper_vel[1] *= -1;
  act.stepper_vel[2] *= -1;
  act.stepper_vel[3] *= -1;

  // convert stepper velocities to xmega timer values
  // 32mhz system clock / 64 = 500khz timer clock
  uint16_t stepper_timers[4];
  for (int i = 0; i < 4; i++)
  {
    if (act.stepper_vel[i] == 0)
      stepper_timers[i] = 0;
    else
    {
      uint32_t t = 200000 / abs(act.stepper_vel[i]);
      if (t > 0x7fff)
        t = 0x7fff;
      if (t < 50)
        t = 50; // don't thrash the mcu with interrupts...
      stepper_timers[i] = t;
      if (act.stepper_vel[i] < 0)
        stepper_timers[i] |= 0x8000; // high bit = direction
    }
  }
  g_stepper_timers[0] = stepper_timers[0];
  g_stepper_timers[1] = stepper_timers[1];
  g_stepper_timers[2] = stepper_timers[2];
  g_stepper_timers[3] = stepper_timers[3];

  for (int i = 0; i < 4; i++)
  {
    uint16_t torque = abs(act.servo_torque[i]);
    if (torque > 1023)
      torque = 1023;
    g_servo_torques[i] = torque;
    g_servo_dirs[i] = (act.servo_torque[i] > 0);
  }
  // flip axes as needed
  g_servo_dirs[0] = !g_servo_dirs[0];
  g_servo_dirs[1] = !g_servo_dirs[1];
  g_servo_dirs[2] = !g_servo_dirs[2];
  g_servo_dirs[3] = !g_servo_dirs[3];
  /*
  printf("%6x %6x %6x %6x\n", stepper_timers[0], stepper_timers[1],
         stepper_timers[2], stepper_timers[3]);
  */
}

/*
void wrist_torques_cb(const openarms::ArmActuators::ConstPtr &msg)
{
  if (msg->torques.size() != NUM_MOTORS)
  {
    ROS_DEBUG("ahhhhhh was expecting %d torques\n", (int)NUM_MOTORS);
    return;
  }
  // blast a packet to each servo 
  for (size_t i = 0; i < NUM_MOTORS; i++)
  {
    double abs_torque = fabs(msg->torques[i] * 1024.0);
    if (abs_torque >= 1024)
      abs_torque = 1023;
    send_torque(i, (uint16_t)floor(abs_torque), (uint8_t)(msg->torques[i] > 0));
  }
}
*/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "armbus");
  ros::NodeHandle n;
  ros::NodeHandle n_private("~");
  std::string port("/dev/ttyUSB0");
  n_private.getParam("port", port);
  n_private.getParam("run_motors", g_run_motors);
  if (!g_run_motors)
    ROS_INFO("You have requested to not run the motors, so I won't.");
  LightweightSerial *s = new LightweightSerial(port.c_str(), 1000000);
  if (!s)
  {
    ROS_FATAL("couldn't open port %s\n", port.c_str());
    ROS_BREAK();
  }
  g_serial = s;
  ros::Subscriber sub = n.subscribe("arm_actuators", 1, actuators_cb);
  ros::Publisher pub = n.advertise<openarms::ArmSensors>("arm_sensors",100);
  ros::Time t_prev = ros::Time::now(), t_wholearm = ros::Time::now();
  for (int i = 0; i < 4; i++)
  {
    g_stepper_timers[i] = 0;
    g_servo_torques[i] = 0;
    g_servo_dirs[i] = 0;
  }
  //ros::spin();
  sensors_msg.pos.resize(8);
  sensors_msg.encoder.resize(4);
  send_stepper_vel(10, 0, 0); // stop em
  send_stepper_vel(11, 0, 0); // stop em
  enable_motor(10, g_run_motors);
  enable_motor(11, g_run_motors);
  enable_motors(g_run_motors);

  uint32_t scheduled = 0;
  bool replied = false;
  const uint32_t SCH_BEGIN             = 0;
  const uint32_t SCH_STEPPER_POS_0     = 0;
  const uint32_t SCH_STEPPER_POS_1     = 1;
  const uint32_t SCH_STEPPER_ACCEL_1   = 2;
  const uint32_t SCH_SERVO_0           = 3;
  const uint32_t SCH_SERVO_1           = 4;
  const uint32_t SCH_SERVO_2           = 5;
  const uint32_t SCH_SERVO_3           = 6;
  const uint32_t SCH_ELBOW_ENCODER     = 7;
  const uint32_t SCH_STEPPER_ENCODER_0 = 8;
  const uint32_t SCH_STEPPER_ENCODER_1 = 9;
  const uint32_t SCH_END               = 9; 
  /*
  for (int i = 1; i <= 4; i++)
    send_joint_limit(i, 0, 0);
  */
  while (n.ok())
  {
    uint8_t read_buf[60];
    int nread = s->read_block(read_buf, sizeof(read_buf)-1);
    if (nread)
    {
      for (int i = 0; i < nread; i++)
        if (process_byte(read_buf[i], &pub))
          replied = true;
    }
    else
      ros::Duration(0.00001).sleep();
    // blast status-request packets periodically
    ros::Time t = ros::Time::now();
    if ((t - t_prev).toSec() > 0.05 || replied)
        //((t - t_prev).toSec() > 0.001 && replied))
    {
      ros::spinOnce();
      //printf("%d\n", scheduled);
      replied = false;
      //enable_led(11, even ? 1 : 0);
      //even = !even;
      //send_stepper_vel(11, 0x7fff, 0x7fff);
      if (scheduled == SCH_STEPPER_POS_0)
      {
        send_stepper_vel(10, g_stepper_timers[0], g_stepper_timers[1]);
        query_stepper_positions(10);
      }
      else if (scheduled == SCH_STEPPER_POS_1)
      {
        //printf("stepper 1 send\n");
        send_stepper_vel(11, g_stepper_timers[2], g_stepper_timers[3]);
        query_stepper_positions(11);
      }
      else if (scheduled == SCH_STEPPER_ENCODER_0)
        query_encoder(10, 2);
      else if (scheduled == SCH_STEPPER_ENCODER_1)
        query_encoder(11, 2);
      else if (scheduled == SCH_STEPPER_ACCEL_1)
        query_accelerometer(11);
      else if (scheduled == SCH_SERVO_0)
      {
        send_torque(1, g_servo_torques[0], g_servo_dirs[0]);
        query_motor(1);
      }
      else if (scheduled == SCH_SERVO_1)
      {
        send_torque(2, g_servo_torques[1], g_servo_dirs[1]);
        query_motor(2);
      }
      else if (scheduled == SCH_SERVO_2)
      {
        send_torque(3, g_servo_torques[2], g_servo_dirs[2]);
        query_motor(3);
      }
      else if (scheduled == SCH_SERVO_3)
      {
        send_torque(4, g_servo_torques[3], g_servo_dirs[3]);
        query_motor(4);
      }
      else if (scheduled == SCH_ELBOW_ENCODER)
        query_encoder(12);
      if (++scheduled > SCH_END)
      {
        double d_wholearm = (t - t_wholearm).toSec();
        static int s_printcount = 0;
        if (s_printcount++ % 100 == 0)
          printf("%.3f hz\n", 1.0 / d_wholearm);
        t_wholearm = t;
        scheduled = SCH_BEGIN;
      }
      t_prev = t;

#if 0
      for (size_t i = 0; i < 1 /*NUM_MOTORS*/; i++)
        query_motor(i);
#endif
    }
  }
  enable_motors(0);
  enable_motor(10, 0); // turn em off
  enable_motor(11, 0); // turn em off
  ros::Duration(0.5).sleep(); // let the shutdown packets get sent out
  delete s;
  return 0;
}

