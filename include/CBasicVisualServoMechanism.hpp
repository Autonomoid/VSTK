#ifndef CBASICVISUALSERVOMECHANISM_H
#define CBASICVISUALSERVOMECHANISM_H

#include <boost/scoped_ptr.hpp>

#include "IServoMechanism.hpp"

namespace vs
{

/**
 * @brief The CBasicVisualServoMechanism class
 */
class CBasicVisualServoMechanism : public IServoMechanism
{
public:
    CBasicVisualServoMechanism();
    ~CBasicVisualServoMechanism();
    void setActuator(IActuator &actuator);
    void setSensor(ISensor& sensor);
    void setController(IController& controller);
    void setDesiredState(const cv::Mat& desiredState);
    void loopOnce();
    cv::Mat getStateError() const;

private:
    class pImpl;
    boost::scoped_ptr<pImpl> mPimpl;
};

} // namespace vs

#endif // CBASICVISUALSERVOMECHANISM_H
