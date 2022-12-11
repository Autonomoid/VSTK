#ifndef IVISIONSENSOR_H
#define IVISIONSENSOR_H

#include "ISensor.hpp"

namespace vs
{

/**
 * @brief The IVisionSensor interface class
 */
class IVisionSensor : public ISensor
{
public:
    virtual void getState(cv::Mat& state) const = 0;
};

} // namespace vs

#endif // IVISIONSENSOR_H
