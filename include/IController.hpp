#ifndef ICONTROLLER_HPP
#define ICONTROLLER_HPP

#include <opencv2/core/core.hpp>

namespace vs
{

/**
 * @brief The IController interface class
 */
class IController
{
public:
    virtual cv::Mat getControlSignal(const cv::Mat& stateError) const = 0;
};

} // namespace vs

#endif // ICONTROLLER_HPP
