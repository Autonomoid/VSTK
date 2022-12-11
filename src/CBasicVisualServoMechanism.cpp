
#include <iostream>

#include "../include/CBasicVisualServoMechanism.hpp"

namespace vs
{

/**
 * @brief The CBasicVisualServoMechanism::pImpl class
 */
class CBasicVisualServoMechanism::pImpl
{
    friend class CBasicVisualServoMechanism;

private:
    IActuator* mActuator;
    IController* mController;
    ISensor* mSensor;
    cv::Mat mDesiredState;
    cv::Mat mStateError;
};


/**
 * @brief CBasicVisualServoMechanism::CBasicVisualServoMechanism
 */
CBasicVisualServoMechanism::CBasicVisualServoMechanism()
    : mPimpl(new CBasicVisualServoMechanism::pImpl)
{}


/**
 * @brief CBasicVisualServoMechanism::~CBasicVisualServoMechanism
 */
CBasicVisualServoMechanism::~CBasicVisualServoMechanism(){}


/**
 * @brief CBasicVisualServoMechanism::getStateError
 * @return
 */
cv::Mat
CBasicVisualServoMechanism::getStateError() const
{
    cv::Mat currentState;
    mPimpl->mSensor->getState(currentState);
    return currentState - mPimpl->mDesiredState;
}


/**
 * @brief CBasicVisualServoMechanism::loopOnce
 */
void
CBasicVisualServoMechanism::loopOnce()
{
    std::cout << "[BasicVisualServoMechanism] Looping once." << std::endl;

    cv::Mat stateError = this->getStateError();
    cv::Mat controlSignal = mPimpl->mController->getControlSignal(stateError);
    mPimpl->mActuator->sendControlSignal(controlSignal);
}


/**
 * @brief CBasicVisualServoMechanism::setActuator
 * @param actuator
 */
void
CBasicVisualServoMechanism::setActuator(IActuator& actuator)
{
    mPimpl->mActuator = &actuator;
}


/**
 * @brief CBasicVisualServoMechanism::setController
 * @param controller
 */
void
CBasicVisualServoMechanism::setController(IController& controller)
{
    mPimpl->mController = &controller;
}


/**
 * @brief CBasicVisualServoMechanism::setDesiredState
 * @param desiredState
 */
void
CBasicVisualServoMechanism::setDesiredState(const cv::Mat& desiredState)
{
    mPimpl->mDesiredState = desiredState;
}


/**
 * @brief CBasicVisualServoMechanism::setSensor
 * @param sensor
 */
void
CBasicVisualServoMechanism::setSensor(ISensor& sensor)
{
    mPimpl->mSensor = &sensor;
}


} // namespace vs
