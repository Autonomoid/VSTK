#include "../../VS/include/CBasicRobotController.hpp"

namespace vs
{

/**
 * @brief The CBasicRobotController::pImpl class
 */
class CBasicRobotController::pImpl
{
    friend class CBasicRobotController;

private:
    void init();
    void updateGain(const cv::Mat& stateError);

    int nDOF;
    cv::Mat mRobotJacobian;
    cv::Mat mInteractionMatrix;
    cv::Mat mTwist;
    cv::Mat mGain;
};


/**
 * @brief CBasicRobotController::CBasicRobotController
 */
CBasicRobotController::CBasicRobotController(int nDOF)
: mPimpl(new CBasicRobotController::pImpl)
{
    mPimpl->nDOF = nDOF;
}


/**
 * @brief CBasicRobotController::~CBasicRobotController
 */
CBasicRobotController::~CBasicRobotController(){}


/**
 * @brief CBasicRobotController::setRobotJacobian
 * @param robotJacobian
 */
void
CBasicRobotController::setRobotJacobian(cv::Mat& robotJacobian)
{
    mPimpl->mRobotJacobian = robotJacobian;
}


/**
 * @brief CBasicRobotController::setInteractionMatrix
 * @param interactionMatrix
 */
void
CBasicRobotController::setInteractionMatrix(cv::Mat& interactionMatrix)
{
    mPimpl->mInteractionMatrix = interactionMatrix;
}


/**
 * @brief CBasicRobotController::setGain
 * @param gain
 */
void CBasicRobotController::setGain(cv::Mat& gain)
{
    mPimpl->mGain = gain;
}


/**
 * @brief CBasicRobotController::setTwist
 * @param twist
 */
void CBasicRobotController::setTwist(cv::Mat& twist)
{
    mPimpl->mTwist = twist;
}


/**
 * @brief CBasicRobotController::getControlSignal
 * @param stateError
 * @return
 */
cv::Mat
CBasicRobotController::getControlSignal(const cv::Mat& stateError) const
{
    cv::Mat J_inv, L_inv;
    cv::invert(mPimpl->mRobotJacobian, J_inv, cv::DECOMP_SVD);
    cv::invert(mPimpl->mInteractionMatrix, L_inv, cv::DECOMP_SVD);


    return -mPimpl->mGain * mPimpl->mTwist * J_inv * L_inv * stateError;
}


} // namespace vs
