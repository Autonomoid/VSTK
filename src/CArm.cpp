#include "../../VS/include/CArm.hpp"

namespace vs
{

/**
 * @brief
 */
CArm::CArm(const ros::NodeHandle& nodeHandle)
    : mModel(new CArmModel(nodeHandle)), mInterface(new CArmInterface(nodeHandle))
{}


/**
 * @brief CArm::setNDOF
 * @param nDOF
 */
void
CArm::setNDOF(const uint nDOF)
{
    mModel->setNDOF(nDOF);
    mInterface->setNDOF(nDOF);
}

} // namespace vs
