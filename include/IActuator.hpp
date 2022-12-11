#ifndef IACTUATOR_HPP
#define IACTUATOR_HPP

#include <opencv2/core/core.hpp>

namespace vs
{

/**
 * @brief The IActuator interface class
 */
class IActuator
{
public:
    virtual void sendControlSignal(cv::Mat& controlSignal) = 0;
};

} // namespace vs

#endif // IACTUATOR_HPP
