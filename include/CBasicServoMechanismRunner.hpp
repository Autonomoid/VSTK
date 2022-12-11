#ifndef CBASICSERVOMECHANISMRUNNER_HPP
#define CBASICSERVOMECHANISMRUNNER_HPP

#include "IServoMechanismRunner.hpp"

namespace vs
{

/**
 * @brief The CBasicServoMechanismRunner class
 */
class CBasicServoMechanismRunner : public IServoMechanismRunner
{
public:
    CBasicServoMechanismRunner();
    virtual void setServoMechanism(vs::IServoMechanism& servoMechanism);
    virtual void setConvergenceThresholds(cv::Mat& convergeThresholds);
    virtual void run();

private:
    vs::IServoMechanism* mServoMechanism;
    cv::Mat mConvergenceThresholds;
};

} // namespace vs

#endif // CBASICSERVOMECHANISMRUNNER_HPP
