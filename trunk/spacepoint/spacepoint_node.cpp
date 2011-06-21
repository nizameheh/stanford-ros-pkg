#include <stdbool.h>
#include <ros/ros.h>
#include "geometry_msgs/Quaternion.h"
#include "libusb-1.0/libusb.h"
using std::string;

#define VID_SPACEPOINT 0x20ff
#define PID_SPACEPOINT 0x0100

/*
bool match_serial_number(struct usb_dev_handle const *usbdev,
                         void *custom, unsigned int len)
{
  string *dev_serial = (string *)custom;
  char *buffer = new char[dev_serial->length()+2];
  // there must be an easier way to do this...
  usb_get_string_simple(const_cast<usb_dev_handle *>(usbdev), usb_device(const_cast<usb_dev_handle *>(usbdev))->descriptor.iSerialNumber, buffer, dev_serial->length()+1);
  buffer[dev_serial->length()+1] = 0;
  bool matched = (strncmp(buffer, dev_serial->c_str(), dev_serial->length()) == 0);
  printf("trying to match %d [%s] with [%s]: %d\n",
         (int)dev_serial->length(), 
         dev_serial->c_str(), buffer, (matched ? 1 : 0));
  delete[] buffer;
  return matched;
}
*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "spacepoint");
  ros::NodeHandle n, n_private("~");
  string serial;
  /*
  HIDInterfaceMatcher matcher;
  matcher.vendor_id = 0x20ff;
  matcher.product_id = 0x0100;
  matcher.matcher_fn = NULL;
  matcher.custom_data = NULL;
  matcher.custom_data_length = 0;
  */
  if (n_private.getParam("serial", serial))
  {
    ROS_INFO("trying to find a spacepoint with serial %s", serial.c_str());
    /*
    matcher.custom_data_length = sizeof(string *);
    matcher.custom_data = &serial;
    matcher.matcher_fn = match_serial_number;
    */
  }
  else
    ROS_INFO("opening whatever spacepoint I can find");

  int dev = 0;
  libusb_context *context = NULL;
  if (libusb_init(&context) < 0)
  {
    ROS_FATAL("libusb init failed\n");
    return 1;
  }
  libusb_device **devices;
  ssize_t num_devices = libusb_get_device_list(context, &devices);
  if (num_devices < 0)
  {
    ROS_FATAL("libusb_get_device_list failed\n");
    return 1;
  }
  printf("%d usb devices\n", (int)num_devices);
  bool found_spacepoint = false;
  for (ssize_t i = 0; i < num_devices; i++)
  {
    libusb_device_descriptor desc;
    if (libusb_get_device_descriptor(devices[i], &desc) < 0)
    {
      ROS_FATAL("libusb_get_device_descriptor failed\n");
      return 1;
    }
    printf("%04x:%04x\n", desc.idVendor, desc.idProduct);
    if (desc.idVendor  == VID_SPACEPOINT &&
        desc.idProduct == PID_SPACEPOINT)
    {
      libusb_device_handle *device;
      if (0 >= libusb_open(devices[i], &device))
      {
        ROS_FATAL("libusb_open failed\n");
        return 1;
      }
      printf("found spacepoint. serial at idx %d\n", desc.iSerialNumber);
      uint8_t serial_buf[1024];
      if (0 >= libusb_get_string_descriptor_ascii(device,
                                                  desc.iSerialNumber,
                                                  serial_buf,
                                                  sizeof(serial_buf)))
      {
        ROS_FATAL("libusb_get_string_descriptor_ascii failed\n");
        return 1;
      }
      printf("serial = [%s]\n", serial_buf);
      found_spacepoint = true;
      libusb_close(device);
    }
  }
  libusb_free_device_list(devices, 1);
  libusb_exit(context);

  if (!found_spacepoint)
  {
    ROS_FATAL("couldn't find spacepoint\n");
    return 1;
  }

  /*
  if (hid_init() != HID_RET_SUCCESS)
  {
    ROS_ERROR("hid_init failed");
    return 1;
  }
  HIDInterface *hid = hid_new_HIDInterface();
  if (!hid)
  {
    ROS_ERROR("hid_new_HIDInterface failed");
    return 1;
  }
  if (hid_force_open(hid, 1, &matcher, 3) != HID_RET_SUCCESS)
  {
    ROS_ERROR("hid_force_open failed");
    return 1;
  }
  */
  ros::Publisher quat_pub = n.advertise<geometry_msgs::Quaternion>("spacepoint_quat", 1);

  uint8_t data[15];
  while (ros::ok())
  {
    /*
    hid_return_t ret = hid_interrupt_read(hid, 0x82, (char *)data, 15, 0);
    if (ret != HID_RET_SUCCESS)
    {
      ROS_ERROR("hid_interrupt_read failed, error %d\n", ret);
      break; // over?
    }
    double val[7];
    for (int i = 0; i < 7; i++)
    {
      int32_t w = (int32_t)(data[i*2]) | ((int32_t)data[i*2+1] << 8);
      w -= 32768;
      val[i] = w * (i < 3 ? 6.0/32768.0 : 1.0/32768.0);
    }
    //uint8_t buttons = data[14];
    // quaternion is stored in 3,4,5,6
    geometry_msgs::Quaternion quat;
    quat.x = val[3];
    quat.y = val[4];
    quat.z = val[5];
    quat.w = val[6];
    quat_pub.publish(quat);
    */

    ros::spinOnce();
  }
  /*
  if (hid_close(hid) != HID_RET_SUCCESS)
  {
    ROS_ERROR("error closing hid device");
    return 1;
  }
  hid_delete_HIDInterface(&hid);
  if (hid_cleanup() != HID_RET_SUCCESS)
  {
    ROS_ERROR("hid_cleanup failed");
    return 1;
  }
  */
  // phew
  ROS_INFO("bai");
  return 0;
}
