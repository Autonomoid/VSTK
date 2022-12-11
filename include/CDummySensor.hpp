#ifndef CDUMMYSENSOR_HPP
#define CDUMMYSENSOR_HPP

#include <iostream>
#include "ISensor.hpp"

namespace vs
{

/**
 * @brief The CDummySensor class
 */
class CDummySensor : public ISensor
{
public:
    CDummySensor(cv::Mat& initialState): mState(initialState){}

    virtual void getState(cv::Mat& state) const
    {
        std::cout << "[DummySensor] Getting state " << mState << std::endl;
        state = mState.clone();
    }

    /**
     * @brief setState is simply to enable the dummySensor to work.
     * @param state
     */
    virtual void setState(cv::Mat state){mState = state;}

private:
    cv::Mat mState;

};

} // namespace vs

#endif // CDUMMYSENSOR_HPP
