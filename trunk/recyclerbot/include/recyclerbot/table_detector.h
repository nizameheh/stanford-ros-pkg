#ifndef TABLEDETECTOR_H
#define TABLEDETECTOR_H

#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/ChannelFloat32.h"
#include "geometry_msgs/Point32.h"
#include "recyclerbot/STANN/sfcnn.hpp"
#include "recyclerbot/STANN/dpoint.hpp"
#include "recyclerbot/STANN/rand.hpp"

using namespace std;

typedef reviver::dpoint<double, 3> Point;

class TableDetector
{
public:
  TableDetector();
  ~TableDetector(){};
	void process_cloud(sensor_msgs::PointCloud& original_msg, sensor_msgs::PointCloud* filtered_msg);
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
		int minDisIter = 0;
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
};

#endif

