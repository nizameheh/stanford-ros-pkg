#include <ros/ros.h>
#include <iostream>
#include "ancient_powercube/ancient_powercube.h"
#include "ancient_powercube/GotoPanTilt.h"
//#include "prosilica_camera/prosilica.h"
#include "polled_camera/GetPolledImage.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
//#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include <string>
#include <stdlib.h>
#include <stdio.h>

using namespace std;

bool image_saved = false;
char filename[100];
sensor_msgs::CvBridge bridge;

void cam_cb(const sensor_msgs::ImageConstPtr &msg)
{
	cout<<"in callback!"<<endl;
  ROS_INFO("Saving image to %s", filename);
  IplImage *cv_image = bridge.imgMsgToCv(msg, "rgb8");
  cvSaveImage(filename, cv_image);
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
	
	ros::ServiceClient cam_client = n.serviceClient<polled_camera::GetPolledImage>("/prosilica/request_image");
	polled_camera::GetPolledImage cam_srv_call;
	cam_srv_call.request.response_namespace = "/prosilica";
	cam_srv_call.request.roi.x_offset = 0;
	cam_srv_call.request.roi.y_offset = 0;
	cam_srv_call.request.roi.height = 200;
	cam_srv_call.request.roi.width = 200;
	ros::Subscriber cam_sub = n.subscribe<sensor_msgs::Image>("/prosilica/image_raw", 1, cam_cb);
	
	// prefix of filenames of files
  string prefix;
  int error = 0;
  cout<<"Input filename prefix: ";
  while (error < 5 && cin>>prefix)
  {
  	for (int i = 0; i < 30 && error < 5; i += angle)
  	{
  		ROS_INFO("Current angle: %d", i);
  		
  		//filename = prefix;
  		//filename.append(itoa(i));
  		sprintf(filename, "/home/jiahui/pr2images/%s%d.jpg", prefix.c_str(), i);
  		
			srv_call.request.pan = (float)i/10;
			if (client.call(srv_call))
				ROS_INFO("Succeed: Move pan angle");
			else
			{
				ROS_ERROR("FAIL: Move pan angle");
				i -= angle;
				error++;
				continue;
			}
			
			if (cam_client.call(cam_srv_call))
				ROS_INFO("Succeed: Take image");
			else
			{
				ROS_ERROR("FAIL: Take image");
				i -= angle;
				error++;
				continue;
			}
			
			ros::Rate rate(1);
			int count = 0;
			while (!image_saved && count++ < 5) rate.sleep();
			
			if (count < 5) ROS_INFO("Succeed: Save image");
			else
			{
				ROS_ERROR("FAIL: Save image");
				i -= angle;
				error++;
				continue;
			}
			error = 0;
			image_saved = false;
		}
  }
  
	return 0;
}

