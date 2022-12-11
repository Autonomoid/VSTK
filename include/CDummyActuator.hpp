#ifndef CDUMMYACTUATOR_HPP
#define CDUMMYACTUATOR_HPP

#include <iostream>
#include "IActuator.hpp"

namespace vs
{

/**
 * @brief The CDummyActuator class
 */
class CDummyActuator : public IActuator
{
public:
    CDummyActuator(){}

    void sendControlSignal(cv::Mat& controlSignal)
    {
        std::cout << "[DummyActuator] Sending control signal" << controlSignal << std::endl;
    }
};

} // namespace vs

#endif // CDUMMYACTUATOR_HPP
