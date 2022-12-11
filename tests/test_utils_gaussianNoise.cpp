/////////////////////////////////////////////////////////////////
//
// Name: test_Utils_gaussianNoise
//
// Test app for utils::gaussianNoise
//

#include "../../VS/include/utils.hpp"

int main(int argc, char const* argv[])
{
  int deviation = 0;

  cvNamedWindow("GaussianNoise", CV_WINDOW_AUTOSIZE);
  cvCreateTrackbar("Deviation","GaussianNoise",&deviation, 500);

  // Set webcam as image source.
  cv::VideoCapture cap(0);
  cv::Mat img;

  while(true)
  {
    // Capture a frame
    cap >> img;

    // Add Gaussian noise.
    vs::utils::gaussianNoise(img, deviation);

    // Display noisey image.
    cv::imshow("GaussianNoise", img);

    cv::waitKey(30);
  }

  return 0;
}
