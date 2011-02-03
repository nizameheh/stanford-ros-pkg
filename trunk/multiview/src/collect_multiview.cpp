#include <ros/ros.h>
#include <iostream>
#include "ancient_powercube/ancient_powercube.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
//#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include <string>

bool image_saved = false;
string filename;
sensor_msgs::CvBridge bridge;

void cam_cb(const sensor_msgs::ImageConstPtr &msg)
{
  IplImage *cv_image = bridge.imgMsgToCv(msg, "rgb8");
  imwrite(filename, cv_image);
  image_saved = true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "collect_multiview");
	
	int angle = 5;
	if (argc == 2)
	{
		angle = atoi(argv[1]);
	}
	
  ros::NodeHandle n;

	ros::ServiceClient client = n.serviceClient<ancient_powercube::GotoPanTilt>("goto_pan_tilt");
	ancient_powercube::GotoPanTilt srv_call;
//	ros::Subscriber powercube_sub = n.subscribe<ancient_powercube::Status>("pantilt_state", 1, pantilt_cb);
	
	ros::ServiceClient cam_client = n.serviceClient<prosilica_camera::GetPolledImage>("/prosilica/request");
	prosilica_camera::GetPolledImage cam_srv_call;
	cam_srv_call.request.response_namespace = "prosilica";
	ros:Subscribe cam_sub = n.subscribe<sensor_msgs::Image>("prosilica/image_raw", 1, cam_cb);
		
  string prefix; // prefix of filenames of files
  while (cin>>prefix)
  {
  	for (int i = 0; i < 360; i += angle)
  	{
  		ROS_INFO("Current angle: %d", i);
  		
  		filename = prefix;
  		filename.append(itoa(i));
  		
			srv_call.request.pan = i;
			if (client.call(srv_call))
				ROS_INFO("Succeed: Move pan angle");
			else
			{
				ROS_ERROR("FAIL: Move pan angle");
				i -= angle;
				continue;
			}
			
			if (cam_client.call(cam_srv_call))
				ROS_INFO("Succeed: Take image");
			else
			{
				ROS_ERROR("FAIL: Take image");
				i -= angle;
				continue;
			}
			
			ros::Rate rate(1);
			int count = 0;
			while (!image_saved && count++ < 10) rate.sleep();
			
			if (count < 10) ROS_INFO("Succeed: Save image");
			else
			{
				ROS_ERROR("FAIL: Save image");
				i -= angle;
				continue;
			}
			
			image_saved = false;
		}
  }
  
	return 0;
}

