#ifndef CARMMODEL_H
#define CARMMODEL_H

#include <vector>

#include <boost/scoped_ptr.hpp>

#include <ros/ros.h>

#include "IRobotModel.hpp"
#include "utils.hpp"


namespace vs
{

class CArmModel : public IRobotModel
{
  public:
    CArmModel(const ros::NodeHandle& nodeHandle);
    ~CArmModel();

    virtual void getForwardKinematics(const cv::Mat& q, cv::Mat& pose) const;
    virtual void getForwardKinematics(cv::Mat& pose) const;

    virtual void getInverseKinematics(const cv::Mat& pose, cv::Mat& q) const;

    virtual void getRobotJacobian(const cv::Mat& q, cv::Mat& jacobian) const;
    virtual void getRobotJacobian(cv::Mat& jacobian) const;

    virtual void get_eJe(const cv::Mat& q, cv::Mat& jacobian) const;

    virtual void get_cJe(const cv::Mat& q, cv::Mat& jacobian) const;
    virtual void get_cJe(cv::Mat& jacobian) const;

    virtual void get_fMe(cv::Mat& fMe) const;

    virtual void setNDOF(const uint n);

    virtual void getJointVelocityFromCameraVelocity(const cv::Mat& cVc, cv::Mat& q_dot) const;

  private:

    class pImpl;
    boost::scoped_ptr<pImpl> mImpl;

};

} // namespace vs

#endif
