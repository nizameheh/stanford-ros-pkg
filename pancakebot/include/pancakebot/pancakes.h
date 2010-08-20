#ifndef PANCAKES_H
#define PANCAKES_H

#include <opencv/cv.h>
#include <opencv/highgui.h>

class PancakeDetector
{
public:
  CvMemStorage *storage, *contour_storage;
  IplImage *color_image, *binary_image;
  //////////////////////////////////////////////////////////////////
  PancakeDetector();
  ~PancakeDetector();
  void process_image(IplImage *cv_image);
};

#endif

