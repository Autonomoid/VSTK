#ifndef IROBOTMODEL_H
#define IROBOTMODEL_H

#include <opencv2/core/core.hpp>

namespace vs
{

class IRobotModel
{
  public:
    virtual void getForwardKinematics(const cv::Mat& q, cv::Mat& pose) const = 0;
    virtual void getForwardKinematics(cv::Mat& pose) const = 0;
    virtual void getInverseKinematics(const cv::Mat& pose, cv::Mat& q) const = 0;
    virtual void getRobotJacobian(const cv::Mat& q, cv::Mat& jacobian) const = 0;
    virtual void setNDOF(const uint n) = 0;
};

} // naemspace vs

#endif
