#include "pancakebot/pancakes.h"
#include "ros/ros.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>

int main(int argc, char **argv)
{
  if (argc != 2)
  {
    printf("usage: pancake_detector_standalone IMAGE\n");
    return 1;
  }
  PancakeDetector pd;
  IplImage *img = cvLoadImage(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
  pd.process_image(img);
  cvWaitKey(0);
  return 0;
}

