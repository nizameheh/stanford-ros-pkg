#include <ros/ros.h>
#include <iostream>
#include "ancient_powercube/ancient_powercube.h"
#include "ancient_powercube/GotoPanTilt.h"
//#include "prosilica_camera/prosilica.h"
#include "polled_camera/GetPolledImage.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include <string>
#include <stdlib.h>
#include <stdio.h>

#define PI 3.1415927

using namespace std;

bool image_saved = false;
char filename[100];
bool initialized = false;

void cam_cb(const sensor_msgs::ImageConstPtr &msg)
{
	if (initialized == false)
	{
		initialized = true;
		return;
	}
	
  ROS_INFO("Saving image to %s", filename);
	sensor_msgs::CvBridge bridge;
  IplImage *cv_image = NULL;

/*  
	ROS_INFO("Encoding: %s", msg->encoding.c_str());
	ROS_INFO("Height: %d", msg->height);
	ROS_INFO("Width: %d", msg->width);
	ROS_INFO("Step: %d", msg->step);*/
	
  // May want to view raw bayer data
  // NB: This is hacky, but should be OK since we have only one image CB.
  if (msg->encoding.find("bayer") != std::string::npos)
      boost::const_pointer_cast<sensor_msgs::Image>(msg)->encoding = "mono8";
  try
  {
    cv_image = bridge.imgMsgToCv(msg, "bgr8");
    cvShowImage("Image window", cv_image);
  }
  catch (sensor_msgs::CvBridgeException error)
  {
    ROS_ERROR("bridge error");
    return;
  }
    
  cvSaveImage(filename, cv_image);
  image_saved = true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "collect_multiview");
	
	cvNamedWindow("Image window");
  cvStartWindowThread();

  ros::NodeHandle n;
  image_transport::ImageTransport it(n);
	image_transport::Subscriber cam_sub = it.subscribe("/prosilica/image_raw", 1, cam_cb);

	int height = 480, width = 640;
  int max_angle = 360, angle = 5;
	n.param("image_height", height, 2*480);
	n.param("image_width", width, 2*640);
  n.param("max_angle", max_angle, 360);
  n.param("angle", angle, 5);
  
	ros::ServiceClient client = n.serviceClient<ancient_powercube::GotoPanTilt>("goto_pan_tilt");
	ancient_powercube::GotoPanTilt srv_call;
//	ros::Subscriber powercube_sub = n.subscribe<ancient_powercube::Status>("pantilt_state", 1, pantilt_cb);
	
	ros::ServiceClient cam_client = n.serviceClient<polled_camera::GetPolledImage>("/prosilica/request_image");
	polled_camera::GetPolledImage cam_srv_call;
	cam_srv_call.request.roi.height = height;
	cam_srv_call.request.roi.width = width;
	cam_srv_call.request.response_namespace = "/prosilica";
	cam_srv_call.request.roi.x_offset = 0;
	cam_srv_call.request.roi.y_offset = 0;

	// ignore the image in buffer
	srv_call.request.pan = 0.0;
	client.call(srv_call);

	// prefix of filenames of files
  string prefix;
  int error = 0;
  
  cout<<"Input filename prefix: ";
  
  while (error < 5 && cin>>prefix)
  {
  	if (prefix == "quit" || prefix == "quit") break;
  	for (int i = 0; i <= max_angle && error < 5; i += angle)
  	{
  		ROS_INFO("Current angle: %d", i);
  		
  		sprintf(filename, "/home/jiahui/pr2images/%s%d.jpg", prefix.c_str(), i);
  		
			srv_call.request.pan = (float)i/180*PI;
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
			
			ros::Rate rate(100);
			int count = 0;
			while (!image_saved && count++ < 500)
			{
				ros::spinOnce();
				rate.sleep();
			}
			
			if (count < 500) ROS_INFO("Succeed: Save image");
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
		srv_call.request.pan = 0;
		ROS_INFO("Moving back to home position");
		if (client.call(srv_call))
			ROS_INFO("Succeed: Move back to home position");
		else
		{
			ROS_ERROR("FAIL: Move pan angle");
		}
		
	  cout<<"Input filename prefix: ";
  }
  
  
	return 0;
}

