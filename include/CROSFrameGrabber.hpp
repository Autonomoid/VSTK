//
// Feed me a ROS topic full of images and I'll make
// you some cv::Mat :D
//


#ifndef CROSFRAMEGRABBER_H
#define CROSFRAMEGRABBER_H

#include <iostream>

#include <boost/scoped_ptr.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include "IVisionSensor.hpp"

namespace vs
{

/**
 * @brief Class to grab image frames from a ROS topic.
 */
class CROSFrameGrabber : public IVisionSensor
{
  public:
    CROSFrameGrabber(ros::NodeHandle nh, std::string& ros_topic);
    ~CROSFrameGrabber();
    void setTopic(std::string rostopic);
    void getState(cv::Mat& state) const;

  private:
    class pImpl;
    boost::scoped_ptr<pImpl> mImpl;
};

} // namespace vs

#endif
