#ifndef ISERVOMECHANISM_H
#define ISERVOMECHANISM_H

#include <opencv2/core/core.hpp>

#include "IActuator.hpp"
#include "ISensor.hpp"
#include "IController.hpp"

namespace vs
{

/**
 * @brief The IServoMechanism interface class
 */
class IServoMechanism
{
public:
    virtual void setActuator(IActuator& actuator) = 0;
    virtual void setSensor(ISensor& sensor) = 0;
    virtual void setController(IController& controller) = 0;
    virtual void setDesiredState(const cv::Mat& desiredState) = 0;
    virtual void loopOnce() = 0;
    virtual cv::Mat getStateError() const = 0;
};

} // namespace vs

#endif // ISERVOMECHANISM_H
