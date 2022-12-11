#ifndef CBASICROBOTCONTROLLER_HPP
#define CBASICROBOTCONTROLLER_HPP

#include <boost/scoped_ptr.hpp>

#include "IController.hpp"

namespace vs
{

/**
 * @brief The CBasicRobotController class
 */
class CBasicRobotController : public IController
{
public:
    CBasicRobotController(int nDOF);
    ~CBasicRobotController();
    void setRobotJacobian(cv::Mat& robotJacobian);
    void setInteractionMatrix(cv::Mat& interactionMatrix);
    void setGain(cv::Mat& gain);
    void setTwist(cv::Mat& twist);
    cv::Mat getControlSignal(const cv::Mat& stateError) const;

private:
    class pImpl;
    boost::scoped_ptr<pImpl> mPimpl;
};

} // namespace vs

#endif // CBASICROBOTCONTROLLER_HPP
