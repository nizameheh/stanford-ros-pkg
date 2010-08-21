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
    unsigned x = cvRound(c[0]), y = cvRound(c[1]), r = cvRound(c[2]);
    // calculate mean and stddev of pixels in this circle
    double pixel_sum = 0, pancake_mean = 0, pancake_stddev = 0;
    int pixel_count = 0, pancake_size = r, pancake_outliers = 0;
    for (int sy = y - r; sy <= y + r; sy++)
      for (int sx = x - r; sx <= x + r; sx++)
      {
        float radius = sqrt((sx-x)*(sx-x) + (sy-y)*(sy-y));
        if (radius > r)
          continue; // we're in a corner of the bounding square
        pixel_sum += CV_IMAGE_ELEM(cv_image, unsigned char, sy, sx);
        pixel_count++;
      }
    pancake_mean = pixel_sum / pixel_count;
    for (int sy = y - r; sy <= y + r; sy++)
      for (int sx = x - r; sx <= x + r; sx++)
      {
        float radius = sqrt((sx-x)*(sx-x) + (sy-y)*(sy-y));
        if (radius > r)
          continue; // we're in a corner of the bounding square
        unsigned char v = CV_IMAGE_ELEM(cv_image, unsigned char, sy, sx);
        float diff = v - pancake_mean;
        pancake_stddev += diff*diff;
      }
    pancake_stddev = sqrt(1.0 / pixel_count * pancake_stddev);
    for (int sy = y - r; sy <= y + r; sy++)
      for (int sx = x - r; sx <= x + r; sx++)
      {
        float radius = sqrt((sx-x)*(sx-x) + (sy-y)*(sy-y));
        if (radius > r)
          continue; // we're in a corner of the bounding square
        unsigned char v = CV_IMAGE_ELEM(cv_image, unsigned char, sy, sx);
        float diff = v - pancake_mean;
        if (fabs(diff) > 40)
          pancake_outliers++;
      }

    uint8_t pancake_grayness = CV_IMAGE_ELEM(cv_image, unsigned char, y, x);
    // dumb classifier... need to convert this to soft constraints
    bool pass = pancake_size > 60   && 
                pancake_size < 100  &&
                pancake_stddev < 30 &&
                pancake_outliers < 2000 &&
                pancake_mean > 50;
    // find connected component
    cvThreshold(cv_image, binary_image, pancake_grayness - 2*pancake_stddev,
                255, CV_THRESH_BINARY);
    /*
    cvFindContours(binary_image, contour_storage, sizeof(CvContour),
                   CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE, cvPoint(0, 0));
    */
    /*
     */
    //for (CvSeq *contour = contour




    cvCircle(color_image, cvPoint(x, y), r,
             (pass ? CV_RGB(0,255,0) : CV_RGB(255,0,0)), 3);


    printf("rad %.0f mean %.1f stddev %.1f %d  x %d y %d\n", c[2],
           pancake_mean, pancake_stddev, pancake_outliers, x, y );
  }
  printf("\n");
  cvShowImage("pancake view", color_image);
  cvWaitKey(5);
}

