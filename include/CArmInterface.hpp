#ifndef CARMINTERFACE_H
#define CARMINTERFACE_H

#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>

#include "IRobotInterface.hpp"

#include "utils.hpp"

namespace vs
{

class CArmInterface : public IRobotInterface
{
public:
    CArmInterface(const ros::NodeHandle& nodeHandle);
    ~CArmInterface();

    virtual void sendControlSignal(cv::Mat &controlSignal);

    virtual void manualDrive();
    virtual void setDOFVelocities(const cv::Mat& dofVelocities);
    virtual void setDOFPositions(const cv::Mat& dofPositions);
    virtual void stop();
    virtual void setNDOF(uint nDOF);

    virtual void getDOFVelocities(cv::Mat& dofVelocities);
    virtual void getDOFPositions(cv::Mat& dofPositions);

private:
    class pImpl;
    boost::scoped_ptr<pImpl> mPimpl;
};

} // namespace vs

#endif // CARMINTERFACE_H
