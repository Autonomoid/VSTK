#ifndef ISERVOMECHANISMRUNNER_HPP
#define ISERVOMECHANISMRUNNER_HPP

#include "IServoMechanism.hpp"

namespace vs
{

/**
 * @brief The IServoMechanismRunner interface class
 */
class IServoMechanismRunner
{
public:
    virtual void setServoMechanism(vs::IServoMechanism& servoMechanism) = 0;
    virtual void setConvergenceThresholds(cv::Mat& convergeThresholds) = 0;
    virtual void run() = 0;
};

} // namespace vs

#endif // ISERVOMECHANISMRUNNER_HPP
