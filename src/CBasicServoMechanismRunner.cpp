#include "../include/CBasicServoMechanismRunner.hpp"

namespace vs
{

CBasicServoMechanismRunner::CBasicServoMechanismRunner()
{}


void
CBasicServoMechanismRunner::setServoMechanism(vs::IServoMechanism& servoMechanism)
{
    mServoMechanism = &servoMechanism;
}


void
CBasicServoMechanismRunner::setConvergenceThresholds(cv::Mat& convergeThresholds)
{
    mConvergenceThresholds = convergeThresholds;
}


void
CBasicServoMechanismRunner::run()
{
    cv::Mat stateError;
    int converged = -1;
    while(converged != 0)
    {
        converged = 0;
        mServoMechanism->loopOnce();

        // Check for convergence.
        stateError = mServoMechanism->getStateError();
        for(int i=0; i<stateError.rows; ++i)
        {
            if(std::abs(stateError.at<double>(i)) > mConvergenceThresholds.at<double>(i))
            {
                ++converged;
            }
        }
    }
}


} // namespace vs
