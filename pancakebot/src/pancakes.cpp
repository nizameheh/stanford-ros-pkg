#include "pancakebot/pancakes.h"
#include <ros/ros.h>

PancakeDetector::PancakeDetector()
{
  cvNamedWindow("pancake view");
  storage = cvCreateMemStorage(0);
  contour_storage = cvCreateMemStorage(0);
  color_image = binary_image = NULL;
}

PancakeDetector::~PancakeDetector()
{
  ROS_INFO("pancake destructor");
  cvDestroyWindow("pancake view");
  cvReleaseMemStorage(&storage);
  cvReleaseMemStorage(&contour_storage);
  if (color_image)
    cvReleaseImage(&color_image);
  if (binary_image)
    cvReleaseImage(&binary_image);
}

void PancakeDetector::process_image(IplImage *cv_image)
{
  //ROS_INFO("checking for pancakes");
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
    /*
       cvFindContours(binary_image, contour_storage, sizeof(CvContour),
       CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE, cvPoint(0, 0));
     */
    //for (CvSeq *contour = contour

    printf("rad %.0f color %d\n", c[2], pancake_grayness);
  }
  cvShowImage("pancake view", color_image);
  cvWaitKey(5);
}

