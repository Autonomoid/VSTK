#include <iostream>
#include "../../VS/include/CLMoments.h"

int main(int  argc, char* argv[])
{
    if(argc == 1)
        exit(1);

    // Threshodling params
    int r_min = 0;
    int r_max = 255;
    int g_min = 0;
    int g_max = 255;
    int b_min = 0;
    int b_max = 255;

    // Create track bars for setting RGB range
    cvNamedWindow("thresholding", CV_WINDOW_AUTOSIZE);
    cvCreateTrackbar("r_min","thresholding",&r_min,255);
    cvCreateTrackbar("r_max","thresholding",&r_max,255);
    cvCreateTrackbar("g_min","thresholding",&g_min,255);
    cvCreateTrackbar("g_max","thresholding",&g_max,255);
    cvCreateTrackbar("b_min","thresholding",&b_min,255);
    cvCreateTrackbar("b_max","thresholding",&b_max,255);

    cv::Mat thresh;
    cv::Mat currentFeatures;

    // Interaction matrix
    cv::Mat L = cv::Mat(6, 6, CV_64F);

    // Read image using path specified at the command-line.
    cv::Mat input = cv::imread(argv[1]);

    while(true)
    {
      // Perform RGB threshold on image
      cv::inRange(input, cv::Scalar(b_min, g_min, r_min), cv::Scalar(b_max, g_max, r_max), thresh);

      // Extract moment features from image.
      vs::utils::getMomentFeatures(thresh, currentFeatures);

      // Display the values of the moment features
      std::cout << "x_g = " << currentFeatures.at<double>(0) << std::endl;
      std::cout << "y_g = " << currentFeatures.at<double>(1) << std::endl;
      std::cout << "a = " << currentFeatures.at<double>(2) << std::endl;
      std::cout << "P_2 = " << currentFeatures.at<double>(3) << std::endl;
      std::cout << "P_3 = " << currentFeatures.at<double>(4) << std::endl;
      std::cout << "theta (deg)= " << currentFeatures.at<double>(5) * 180.0 / M_PI << std::endl << std::endl;

      // Compute the interaction matrix for the moment features
      CLMoments lmoments;
      double order  = 3;
      lmoments.setImage<cv::Mat>(thresh, order);

      // Ax + By + C = 1/Z
      double A = 0;
      double B = 0;
      double C = 0.25; // Z = 0.25 m

      lmoments.setPlane(A, B, C);
      L = lmoments.getInteractionMatrix();
      std::cout << "Interaction matrix:" << std::endl << L << std::endl;

      // Draw the CoM on the image.
      cv::Point2f p(currentFeatures.at<double>(0), currentFeatures.at<double>(1));
      cv::Mat decorated = input.clone();
      cv::circle(decorated, p, 5, cv::Scalar(255,0,255), 1, 8);
      cv::imshow("thresholded", thresh);
      cv::imshow("input", decorated);
      cv::waitKey(30);
    }

    return 0;
}
