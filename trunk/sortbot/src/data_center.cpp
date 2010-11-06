//write a ros node to begin analysis for sortbot
//goals:
//create a ros node
// use rostopic list to see a list of all messages
// 1. subscribe to /narrow_stereo_textured/points2_drop
// 2. read out the list of points focus on examples show in 
//gedit  include/sensor_msgs/point_cloud_conversion.h 
// 3. for how to loop through this message - possibly look to other software
//
// 4. read data points in from the topic /narrow_stereo_textured/points2_drop
// 5. compute a histogram in z
// 6. fit a plane to points that fall in the biggest bin (optional ?)
// 7. publish a msg to rosrun rviz rviz[format=wiki/rviz/DisplayTypes/Marker]
// 8. display a thin box representing the points in rviz

//includes
#include <ros/ros.h>
#include <sensory_msgs/PointCloud2.h>
#include <stdio>  
#include <algorithm>

using namespace std;

int sensor_msgs::getPointCloud2FieldIndex (const sensor_msgs::PointCloud2 &cloud, const std::string &field_name){
  // Get the index we need
  // this function from 
  // $Id: point_cloud_conversion.h 32910 2010-09-27 22:48:42Z tfoote $

  for (size_t d = 0; d < cloud.fields.size (); ++d)
    if (cloud.fields[d].name == field_name)
      return (d);
  return (-1);
}


void readPointsCallback(const sensor_msgs::PointCloud2 &input){
//input = point cloud
// process data (steps 2-7) here - write helper functions as necessary
  vector<float> z_data;
  z_data.resize(input.width*input.height);
   
  //Get the x/y/z field offsets
  int x_idx = getPointCloud2FieldIndex(input, "x");
  int y_idx = getPointCloud2FieldIndex(input, "y");
  int z_idx = getPointCloud2FieldIndex(input, "z");

  if (x_idx == -1 || y_idx == -1 || z_idx == -1){
    ROS_ERROR ("x/y/z coordinates not found! Cannot convert to sensor_msgs::PointCloud!");
    return (false);
  }
  int x_offset = input.fields[x_idx].offset;
  int y_offset = input.fields[y_idx].offset;
  int z_offset = input.fields[z_idx].offset;

  //Copy the data points
  for(size_t cp = 0; cp < z_data.size(); ++cp){
  // in conversion program he has ++cp which increments before running (right?)
  // should this be (cp ++) or is the data indexed from 1?
    
    // Copy x/y/z - keep only z
    memcpy( &z_data[cp], &input.data[cp * input.point_step + z_offset], sizeof(float));
  }
  
  //assume we correctly have z_data - output some stuff
  float biggest = -1000000.0;
  float smallest = 1000000.0;
  for(int iZ = 0; iZ < z_data.size(); iZ++){
    if(z_data[iZ]<smallest){ 
      smallest = z_data[iZ];
    }
    if(z_data[iZ]>biggest){
      biggest = z_data[iZ];
    }
  }
  printf("z_min = %f, z_max = %f\n", smallest, biggest);
}

int main(int argc, char**argv){
  
  ros::NodeHandle  nh; //the trailing "_" means internal variable of a class
  ros::Subscriber stereo_pts;
  ros::Publisher z_plane;
  //vector<float> z_data;
  //float z_level;
  int queue_size = 100;
  
//SUBSCRIBE TO /narrow/stereo/textured/points2_drop
  stereo_pts = nh.subscribe("/narrow_stereo_textured/points2_drop",queue_size,readPointsCallback);
  //


}
