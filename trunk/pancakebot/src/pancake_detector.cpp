#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>

class PancakeDetector
{
public:
  ros::NodeHandle n;
  image_transport::ImageTransport it;
  image_transport::Subscriber image_sub;
  image_transport::Publisher image_pub;
  sensor_msgs::CvBridge bridge;
  CvMemStorage *storage, *contour_storage;
  IplImage *color_image, *binary_image;
  //////////////////////////////////////////////////////////////////
  PancakeDetector(ros::NodeHandle &_n) : n(_n), it(_n)
  {
    cvNamedWindow("pancake cam");
    storage = cvCreateMemStorage(0);
    contour_storage = cvCreateMemStorage(0);
    image_sub = it.subscribe("image", 1, &PancakeDetector::image_cb, this);
    color_image = binary_image = NULL;
  }
  ~PancakeDetector()
  {
    ROS_INFO("pancake destructor");
    cvDestroyWindow("pancake cam");
    cvReleaseMemStorage(&storage);
    cvReleaseMemStorage(&contour_storage);
    if (color_image)
      cvReleaseImage(&color_image);
    if (binary_image)
      cvReleaseImage(&binary_image);
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
    if (!color_image)
    {
      color_image = cvCreateImage(cvGetSize(cv_image), IPL_DEPTH_8U, 3);
      binary_image = cvCreateImage(cvGetSize(cv_image), IPL_DEPTH_8U, 1);
    }
    cvSmooth(cv_image, cv_image, CV_GAUSSIAN, 9, 9);
    cvCvtColor(cv_image, color_image, CV_GRAY2RGB);
    cvClearMemStorage(storage);
    cvClearMemStorage(contour_storage);
    CvSeq *seq = cvHoughCircles(cv_image, storage, CV_HOUGH_GRADIENT,
                                2, // accumulator resolution
                                80, // minimum spacing
                                100, 90, // canny, hough thresholds
                                50, 150); // min, max radius
    for (int i = 0; i < seq->total; i++)
    {
      float *c = (float *)cvGetSeqElem(seq, i);
      unsigned x = cvRound(c[0]), y = cvRound(c[1]);
      cvCircle(color_image, cvPoint(x, y),
               cvRound(c[2]), CV_RGB(255,0,0), 3);
      uint8_t pancake_grayness = CV_IMAGE_ELEM(cv_image, unsigned char, y, x);
      // find connected component
      cvThreshold(cv_image, binary_image, pancake_grayness-20, 255,
                  CV_THRESH_BINARY);
      cvFindContours(binary_image, contour_storage, sizeof(CvContour),
                     CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE, cvPoint(0, 0));
      for (CvSeq *contour = contour

      printf("rad %.0f color %d\n", c[2], pancake_grayness);
                                 
    }
    cvShowImage("pancake cam", color_image);
    cvWaitKey(5);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pancake_detector");
  ros::NodeHandle n;
  PancakeDetector pd(n);
  ros::spin();
  return 0;
}

