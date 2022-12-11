#ifndef CDUMMYCONTROLLER_H
#define CDUMMYCONTROLLER_H

#include <iostream>
#include "IController.hpp"

namespace vs
{

/**
 * @brief The CDummyController class
 */
class CDummyController : public IController
{
public:
    CDummyController() : mGain(0.0) {}

    void setGain(const double gain)
    {
        std::cout << "[DummyController] Setting gain to " << gain << std::endl;
        mGain = gain;
    }

    cv::Mat getControlSignal(const cv::Mat& stateError) const
    {
        std::cout << "[DummyController] Getting control signal " << (mGain * stateError) << std::endl;
        return mGain * stateError;
    }

private:
    double mGain;
};

} // namespace vs

#endif // CDUMMYCONTROLLER_H
