#ifndef IROBOTINTERFACE_HPP
#define IROBOTINTERFACE_HPP

#include <opencv2/core/core.hpp>

#include "../../VS/include/IActuator.hpp"

namespace vs
{

class IRobotInterface : public IActuator
{
public:

    virtual void manualDrive() = 0;
    virtual void setDOFVelocities(const cv::Mat& dofVelocities) = 0;
    virtual void setDOFPositions(const cv::Mat& dofPositions) = 0;
    virtual void stop() = 0;
    virtual void setNDOF(const uint nDOF) = 0;

    virtual void getDOFVelocities(cv::Mat& dofVelocities) = 0;
    virtual void getDOFPositions(cv::Mat& dofPositions) = 0;
};

} // namespace vs

#endif // IROBOTINTERFACE_HPP
