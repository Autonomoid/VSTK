#ifndef CLOOPBASIC_H
#define CLOOPBASIC_H

#include "ILoop.h"

class CLoopBasic : public ILoop
{
public:
    CLoopBasic();
    void loopOnce(ITarget&);
    void setController(IController&);
    void setFeatureExtractor(IFeatureExtractor&);
    void setRobot(IRobot&);
    void setSource(ISource&);
    state getState() const;

private:
    IController m_controller;
    IFeatureExtractor m_featureExtractor;
    IRobot m_robot;
    ISource m_source;
    ITarget m_target;
    state m_state;
};

#endif // CLOOPBASIC_H
