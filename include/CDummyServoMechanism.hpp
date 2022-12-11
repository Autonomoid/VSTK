#ifndef CDUMMYSERVOMECHANISM_H
#define CDUMMYSERVOMECHANISM_H

#include "IServoMechanism.hpp"

namespace vs
{

/**
 * @brief The CDummyServoMechanism class
 */
class CDummyServoMechanism : public IServoMechanism
{
public:
    CDummyServoMechanism();
    void setActuator(IActuator& actuator);
    void setSensor(ISensor& sensor);
    void setController(IController &controller);
    void setDesiredState(const cv::Mat& desiredState);
    void loopOnce();
    cv::Mat getStateError() const;

private:
    IActuator* mActuator;
    IController* mController;
    ISensor* mSensor;
    cv::Mat mDesiredState;
    cv::Mat mStateError;
};

} // namespace vs

#endif // CDUMMYSERVOMECHANISM_H
