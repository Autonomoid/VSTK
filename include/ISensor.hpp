#ifndef ISENSOR_HPP
#define ISENSOR_HPP

#include <opencv2/core/core.hpp>

namespace vs
{

/**
 * @brief The ISensor interface class
 */
class ISensor
{
public:
    virtual void getState(cv::Mat & state) const = 0;
};

} // namespace vs

#endif // ISENSOR_HPP
