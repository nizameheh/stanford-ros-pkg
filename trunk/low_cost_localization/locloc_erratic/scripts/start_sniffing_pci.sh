#!/bin/bash
source /opt/ros/cturtle/setup.sh
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/brian/ros/stanford-ros-pkg
rosrun wifi_sniffer wifi_sniffer_node __name:=wifi_sniffer_pci _iface:=wlan0 wifi_scan:=wifi_scan_pci
