#include "pancakebot/pancakes.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>

class PancakeDetectorNode
{
public:
  ros::NodeHandle n;
  image_transport::ImageTransport it;
  image_transport::Subscriber image_sub;
  image_transport::Publisher image_pub;
  sensor_msgs::CvBridge bridge;
  PancakeDetector detector;
  //////////////////////////////////////////////////////////////////
  PancakeDetectorNode(ros::NodeHandle &_n) : n(_n), it(_n)
  {
    image_sub = it.subscribe("image", 1, &PancakeDetectorNode::image_cb, this);
  }
  ~PancakeDetectorNode()
  {
    ROS_INFO("PancakeDetectorNode destructor");
  }
  void image_cb(const sensor_msgs::ImageConstPtr &msg)
  {
    //ROS_INFO("checking for pancakes");
    IplImage *cv_image = NULL;
    try
    {
      cv_image = bridge.imgMsgToCv(msg, "mono8");
    }
    catch (sensor_msgs::CvBridgeException error)
    {
      ROS_ERROR("bridge error");
      return;
    }
    detector.process_image(cv_image);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pancake_detector");
  ros::NodeHandle n;
  PancakeDetectorNode pdn(n);
  ros::spin();
  return 0;
}

