#include "CLoopBasic.h"

namespace vs
{

CLoopBasic::CLoopBasic()
{
}

void
CLoopBasic::loopOnce(ITarget&)
{
    //this->m_source;
    //this->m_featureExtractor;
}

state
CLoopBasic::getState() const
{
    return this->m_state;
}

void
CLoopBasic::setController(IController &controller)
{
    this->m_controller = controller;
}

void
CLoopBasic::setFeatureExtractor(IFeatureExtractor &featureExtractor)
{
    this->m_featureExtractor = featureExtractor;
}

void
CLoopBasic::setRobot(IRobot &robot)
{
    this->m_robot = robot;
}

void
CLoopBasic::setSource(ISource &source)
{
    this->m_source = source;
}

}
