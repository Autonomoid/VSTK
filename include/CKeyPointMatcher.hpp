/////////////////////////////////////////////////////////////////
//
// Name: CKeyPointMatcher
//
// Based on the RobustMatcher class described in chapter 9 of:
// "OpenCV2 Computer Vision Application Programming Cookbook"
//

#ifndef CKEYPOINTMATCHER
#define CKEYPOINTMATCHER

#include <iostream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/nonfree.hpp> // For playing with SIFT + SURF :D

typedef std::vector<cv::DMatch> vecDMatch_t;
typedef std::vector<std::vector<cv::DMatch> > vec2DMatch_t;
typedef std::vector<cv::KeyPoint> vecKeyPoint_t;


/**
 * @brief 
 */
class CKeyPointMatcher
{
  public:
    CKeyPointMatcher();
    void setAlgorithm(const std::string kp = "ORB");
    void setFeatureDetector(const cv::Ptr<cv::FeatureDetector>&);
    void setDescriptorExtractor(const cv::Ptr<cv::DescriptorExtractor>&);
    void setDescriptorMatcher(const cv::Ptr<cv::DescriptorMatcher>&);
    void setConfidenceLevel(const double);
    void setMinDistanceToEpipolar(const double);
    void setRatio(const float);
    int ratioTest(vec2DMatch_t&);
    void symmetryTest(const vec2DMatch_t&, const vec2DMatch_t&, vecDMatch_t&);
    void ROIinclude(const vecDMatch_t&, vecDMatch_t&, const vecKeyPoint_t&, const vecKeyPoint_t&, const cv::Point, const cv::Point);
    cv::Mat ransacTest(const vecDMatch_t&, const vecKeyPoint_t&, const vecKeyPoint_t&, vecDMatch_t&);
    cv::Mat match(cv::Mat&, cv::Mat&, vecDMatch_t&, vecKeyPoint_t&, vecKeyPoint_t&, const cv::Point, const cv::Point);
    cv::Mat match(cv::Mat&, cv::Mat&, vecDMatch_t&, vecKeyPoint_t&, vecKeyPoint_t&);
    std::string current_alg;

  private:
    cv::Ptr<cv::FeatureDetector> detector;
    cv::Ptr<cv::DescriptorExtractor> extractor;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    float ratio; // Ratio test -- max ratio between 1st and 2nd NN
    bool refineF; // if true will refine the F matrix
    double distance; // RANSAC test -- min distance to epipolar
    double confidence; // RANSAC test -- confidence level (probability)

    std::map<std::string, int> kpAlgorithm; // Available algorithms.
};

#endif
