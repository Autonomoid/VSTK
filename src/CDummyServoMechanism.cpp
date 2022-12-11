#include <iostream>

#include "../include/CDummyServoMechanism.hpp"
#include "../include/CDummySensor.hpp"

namespace vs
{

/**
 * @brief CDummyServoMechanism::CDummyServoMechanism
 */
CDummyServoMechanism::CDummyServoMechanism()
{}


/**
 * @brief CDummyServoMechanism::getStateError
 * @return
 */
cv::Mat
CDummyServoMechanism::getStateError() const
{
    cv::Mat currentState;
    mSensor->getState(currentState);
    return currentState - mDesiredState;
}


/**
 * @brief CDummyServoMechanism::loopOnce
 */
void
CDummyServoMechanism::loopOnce()
{
    std::cout << "[DummyServoMechanism] Looping once." << std::endl;

    cv::Mat stateError = this->getStateError();
    cv::Mat controlSignal = mController->getControlSignal(stateError);
    mActuator->sendControlSignal(controlSignal);

    // Update the dummy state of the sensor.
    cv::Mat currentState;
    mSensor->getState(currentState);
    cv::Mat newState = currentState + mController->getControlSignal(stateError);
    dynamic_cast<CDummySensor*>(mSensor)->setState(newState);
}


/**
 * @brief CDummyServoMechanism::setActuator
 * @param actuator
 */
void
CDummyServoMechanism::setActuator(IActuator& actuator)
{
    mActuator = &actuator;
}


/**
 * @brief CDummyServoMechanism::setController
 * @param controller
 */
void
CDummyServoMechanism::setController(IController& controller)
{
    mController = &controller;
}


/**
 * @brief CDummyServoMechanism::setDesiredState
 * @param desiredState
 */
void
CDummyServoMechanism::setDesiredState(const cv::Mat& desiredState)
{
    std::cout << "[DummyServoMechanism] Setting desired state to " << desiredState << std::endl;
    mDesiredState = desiredState;
}


/**
 * @brief CDummyServoMechanism::setSensor
 * @param sensor
 */
void
CDummyServoMechanism::setSensor(ISensor& sensor)
{
    mSensor = &sensor;
}


} // namespace vs
