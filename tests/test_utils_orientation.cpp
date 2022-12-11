/////////////////////////////////////////////////////////////////
//
// Name: test_Utils_orientation
//
// Test app for utils::orientation
//

#define DEBUG

#ifdef DEBUG
    #define dbg(msg) std::cout << "[DBG] " << __TIME__ << ":" << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": " << msg << std::endl;
#else
    #define dbg(msg)
#endif

#include <iostream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../../VS/include/utils.hpp"

int main(int argc, const char *argv[])
{
    if(argc < 2)
    {
      std::cout << "USAGE: program_name <IMAGE PATH>" << std::endl;
      return 1;
    }

    cv::Mat im = cv::imread(argv[1]);    

    // Check for empty image
    if(im.empty()==true)
    {
      std::cout << "Failed to read image!" << std::endl;
      return 1;
    }

    double orient = vs::utils::orientation(im);

    std::cout << " Image orientation = " << orient * (180.0/3.14159265358) << " deg" << std::endl;

    cv::Mat G;
    cv::cvtColor(im, G, CV_BGR2GRAY);
    cv::imshow("Intensity image", G);
    cv::waitKey(); 
    
    return 0;
}

