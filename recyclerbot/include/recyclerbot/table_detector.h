#ifndef TABLEDETECTOR_H
#define TABLEDETECTOR_H

#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/ChannelFloat32.h"
#include "geometry_msgs/Point32.h"

class TableDetector
{
public:
  TableDetector();
  ~TableDetector(){};
	void process_cloud(sensor_msgs::PointCloud& original_msg, sensor_msgs::PointCloud* filtered_msg);
};

#endif

