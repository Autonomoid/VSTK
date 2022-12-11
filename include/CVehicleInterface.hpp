#ifndef CTRIDENT_H
#define CTRIDENT_H

#include <boost/scoped_ptr.hpp>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include "IRobotInterface.hpp"

namespace vs
{

class CVehicleInterface : public IRobotInterface
{
public:
    CVehicleInterface(const ros::NodeHandle& nodeHandle);
    ~CVehicleInterface();

    virtual void sendControlSignal(cv::Mat& controlSignal);

    virtual void manualDrive();
    virtual void setDOFVelocities(const cv::Mat& dofVelocities);
    virtual void setDOFPositions(const cv::Mat& dofPositions);
    virtual void stop();
    virtual void setNDOF(const uint nDOF);
    virtual void setHomePose(const cv::Mat& homePose);

    virtual void getDOFVelocities(cv::Mat& dofVelocities);
    virtual void getDOFPositions(cv::Mat& dofPositions);

private:
    class pImpl;
    boost::scoped_ptr<pImpl> mPimpl;
};

} // namespace vs

#endif
