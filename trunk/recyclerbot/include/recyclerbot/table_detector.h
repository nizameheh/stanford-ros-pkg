#ifndef TABLEDETECTOR_H
#define TABLEDETECTOR_H

#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/ChannelFloat32.h"
#include "geometry_msgs/Point32.h"
#include "visualization_msgs/Marker.h"
#include "recyclerbot/STANN/sfcnn.hpp"
#include "recyclerbot/STANN/dpoint.hpp"
#include "recyclerbot/STANN/rand.hpp"
#include "math.h"

using namespace std;

typedef reviver::dpoint<double, 3> Point;

class TableDetector
{
public:
  TableDetector();
  ~TableDetector(){};
	void process_cloud(sensor_msgs::PointCloud& original_msg,
	                   sensor_msgs::PointCloud* filtered_msg,
	                   vector<visualization_msgs::Marker>& marker_msg);
	void find_cluster(vector<geometry_msgs::Point32>& pointCloud, 
										int k, 
										vector<long unsigned int>& clusterId)
	{
		int i = 0;
		vector<Point> points;
		Point p_;
		vector<double> distance;
		int size = pointCloud.size();
		int iter = 10;
		int startPoint = 0;
		
		for (i = 0; i < size; i++)
		{
			p_[0] = pointCloud[i].x;
			p_[1] = pointCloud[i].y;
			p_[2] = pointCloud[i].z;
			points.push_back(p_);
		}
		
		sfcnn<Point, 3, double>
		  NN(&points[0], size);

		int minDistance = 1000;
//		int minDisIter = 0;
		vector<long unsigned int> tempCluster;
		
		for (i = 0; i < iter; i++)
		{
			startPoint = int(__drand48__() * (size - 1));
			NN.ksearch(points[startPoint], k, tempCluster, distance);
			if (distance.back() < minDistance)
			{
				minDistance = distance.back();
				clusterId = tempCluster;
				tempCluster.clear();
			}
		}
		//for (i = 0; i < 20; i++) cout<<answer[i]<<"; ";
	}
	
	void find_cylinder(vector<geometry_msgs::Point32>& pointCloud,
									   geometry_msgs::Point32& center,
									   double& r,
									   double& h,
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

