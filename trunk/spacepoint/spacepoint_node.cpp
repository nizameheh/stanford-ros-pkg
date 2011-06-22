#include <stdbool.h>
#include <ros/ros.h>
#include <linux/hidraw.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <libudev.h>
#include "geometry_msgs/Quaternion.h"
using std::string;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "spacepoint");
  ros::NodeHandle n, n_private("~");
  string serial;
  if (n_private.getParam("serial", serial))
  {
    // workaround for command-line parameter type detection... start w/space
    if (serial[0] == ' ')
      serial = serial.substr(1);
    ROS_INFO("trying to open a spacepoint with serial %s", serial.c_str());
  }
  else
  {
    ROS_FATAL("please provide a serial param for the device\n");
    return 1;
  }

  string hidraw_path;
  udev *udev;
  udev_enumerate *enumerate;
  udev_list_entry *udev_devices, *udev_dev_list_entry;
  udev_device *udev_dev;
  udev = udev_new();
  if (!udev)
  {
    ROS_FATAL("couldn't open libudev\n");
    return 1;
  }
  enumerate = udev_enumerate_new(udev);
  udev_enumerate_add_match_subsystem(enumerate, "hidraw");
  udev_enumerate_scan_devices(enumerate);
  udev_devices = udev_enumerate_get_list_entry(enumerate);
  bool found = false;
  udev_list_entry_foreach(udev_dev_list_entry, udev_devices) {
    if (found)
      continue;
    const char *path;
    path = udev_list_entry_get_name(udev_dev_list_entry);
    udev_dev = udev_device_new_from_syspath(udev, path);

    string kernel_name = udev_device_get_sysname(udev_dev);
    //udev_device *hid_dev = udev_device_get_parent_with_subsystem_devtype(udev_dev, "usb", "usbhid");
    udev_device *hid_dev = udev_device_get_parent(udev_device_get_parent(udev_dev));

    udev_dev = udev_device_get_parent_with_subsystem_devtype(udev_dev, "usb", "usb_device");
    //printf("  vid/pid = %s %s\n", udev_device_get_sysattr_value(udev_dev, "idVendor"), udev_device_get_sysattr_value(udev_dev, "idProduct"));
    //printf(" exploring %s\n", udev_list_entry_get_name(udev_dev_list_entry));
    if (!strncmp(udev_device_get_sysattr_value(udev_dev,"idVendor"),"20ff",4) &&
        !strncmp(udev_device_get_sysattr_value(udev_dev,"idProduct"),"0100",4))
    {
      const char *ifnum = udev_device_get_sysattr_value(hid_dev,"bInterfaceNumber");
      if (!ifnum)
        ROS_ERROR("bad ifnum string");
      //ROS_INFO("ifnum = %s\n", ifnum);
      if (strcmp(ifnum, "01"))
        continue;
      //printf("pid/vid match\n");
      const char *dev_ser = udev_device_get_sysattr_value(udev_dev, "serial");
      //printf("  [%s]\n", dev_ser);
      if (!strcmp(dev_ser, serial.c_str()))
      {
        //printf("found at %s -> %s\n", path, udev_device_get_devnode(udev_dev));
        hidraw_path = string("/dev/") + kernel_name; //string(path); //string(udev_device_get_devnode(udev_dev));
        found = true;
      }
    }
    udev_device_unref(udev_dev);
    //if (hid_dev)
    //  udev_device_unref(hid_dev);
  }
  udev_enumerate_unref(enumerate);
  udev_unref(udev);

  if (!found)
  {
    ROS_FATAL("couldn't find spacepoint serial %s\n", serial.c_str());
    return 1;
  }
  ROS_INFO("opening %s", hidraw_path.c_str());
  //hidraw_path = "/dev/hidraw8";

  int fd = open(hidraw_path.c_str(), O_RDONLY); //|O_NONBLOCK);
  if (fd < 0)
  {
    ROS_FATAL("couldn't open %s\n", hidraw_path.c_str());
    return 1;
  }
  ros::Publisher quat_pub = n.advertise<geometry_msgs::Quaternion>("spacepoint_quat", 1);
  uint8_t data[150];
  while (ros::ok())
  {
    int res = read(fd, data, sizeof(data));
    if (res < 0)
    {
      ROS_FATAL("bad read");
      perror("read");
    }
    else
    {
      //printf("read() got %d bytes\n", res);
      if (res == 15)
      {
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
      }
    }
    /*
    hid_return_t ret = hid_interrupt_read(hid, 0x82, (char *)data, 15, 0);
    if (ret != HID_RET_SUCCESS)
    {
      ROS_ERROR("hid_interrupt_read failed, error %d\n", ret);
      break; // over?
    }
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
  close(fd);
  return 0;
}
