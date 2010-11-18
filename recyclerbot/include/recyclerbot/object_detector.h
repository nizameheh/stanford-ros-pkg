#ifndef OBJECTDETECTOR_H
#define OBJECTDETECTOR_H

#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/ChannelFloat32.h"
#include "geometry_msgs/Point32.h"
#include "visualization_msgs/Marker.h"
#include "recyclerbot/STANN/sfcnn.hpp"
#include "recyclerbot/STANN/dpoint.hpp"
#include "recyclerbot/STANN/rand.hpp"
#include "math.h"

#define MAX_CYL_NUM 10
#define CYL_POINT_THRESH 3500
#define COLORNUM 10

float color_array[COLORNUM][4] = {
{1.0, 0.0, 0.0, 1.0}, // red
{0.0, 1.0, 0.0, 1.0}, // green
{0.0, 0.0, 1.0, 1.0}, // blue
{1.0, 1.0, 0.0, 1.0}, // ?
{0.0, 1.0, 1.0, 1.0}, // ?
{1.0, 0.0, 1.0, 1.0}, // ?
{0.5, 0.8, 0.2, 1.0}, // ?
{0.2, 0.5, 0.8, 1.0}, // ?
{0.8, 0.2, 0.5, 1.0}, // ?
{0.5, 0.5, 0.5, 1.0}  // ?
};


using namespace std;

typedef reviver::dpoint<double, 3> Point;

struct cylinder
{
	geometry_msgs::Point32 center;
	double radius;
	double height;
	int pointNum;
	int classId; // 0 - can, 1 - glass bottle, 2 - plastic bottle
};

class ObjectDetector
{
public:
	std_msgs::ColorRGBA colorArray[COLORNUM];
	
  ObjectDetector();
  ~ObjectDetector(){};
	void process_cloud(sensor_msgs::PointCloud& nt_msg,
										 sensor_msgs::PointCloud& ws_msg,
										 int& cylNum,
	                   vector<sensor_msgs::PointCloud>& filtered_msgs,
	                   vector<visualization_msgs::Marker>& marker_msg);
	void find_cluster(vector<geometry_msgs::Point32>& pointCloud, 
										int k, 
										vector<long unsigned int>& clusterId);
	
	void find_cylinder(vector<geometry_msgs::Point32>& pointCloud,
										 vector<long unsigned int>& preclusterId,
									   cylinder& cylinder_,
										 vector<long unsigned int>& clusterId);
										 
	void find_circle(geometry_msgs::Point32& p1, geometry_msgs::Point32& p2, geometry_msgs::Point32& p3,
									 geometry_msgs::Point32& center, double& r)
	{
		double delta = (p3.x - p2.x) * (p2.y - p1.y) - (p2.x - p1.x) * (p3.y - p2.y);
		double ss1 = p1.x * p1.x + p1.y * p1.y;
		double ss2 = p2.x * p2.x + p2.y * p2.y;
		double ss3 = p3.x * p3.x + p3.y * p3.y;
		center.x = ((p3.y - p2.y) * ss1 + (p1.y - p3.y) * ss2 + (p2.y - p1.y) * ss3) / (2 * delta);
		center.y = ((p3.x - p2.x) * ss1 + (p1.x - p3.x) * ss2 + (p2.x - p1.x) * ss3) / (2 * delta);
		r = sqrt((center.x - p1.x)*(center.x - p1.x) + (center.y - p1.y)*(center.y - p1.y));
	}
	
	double distance(geometry_msgs::Point32& p1, geometry_msgs::Point32& p2)
	{
		return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
	}
	
	void fit_circle(vector<geometry_msgs::Point32>& pointCloud,
									geometry_msgs::Point32& center,
									double& r);
	
};

#endif

