/////////////////////////////////////////////////////////////////
//
// Name: test_Utils_motionBlur
//
// Test app for utils::motionBlur
//

#include "../../VS/include/utils.hpp"

int main(int argc, char const* argv[])
{
  int size = 3;
  int direction = 0;

  cvNamedWindow("MotionBlur", CV_WINDOW_AUTOSIZE);
  cvCreateTrackbar("Kernel size","MotionBlur",&size,200);
  cvCreateTrackbar("Direction","MotionBlur",&direction,360);

  // Set webcam as image source.
  cv::VideoCapture cap(0);
  cv::Mat img;

  while(true)
  {
    // Capture a frame
    cap >> img;

    // Blur image
    vs::utils::motionBlur(img, size, direction);

    // Display blurred image
    cv::imshow("MotionBlur", img);

    cv::waitKey(30);
  }

  return 0;
}
