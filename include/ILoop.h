#ifndef ILOOP_H
#define ILOOP_H

#include "vs.h"
#include "IController.h"
#include "IFeatureExtractor.h"
#include "IRobot.h"
#include "ISource.h"
#include "ITarget.h"

namespace vs
{

class ILoop
{
public:
    virtual ~ILoop(){}
    virtual void loopOnce(ITarget&) = 0;
    virtual void setControler(IController&) = 0;
    virtual void setFeatureExtractor(IFeatureExtractor&) = 0;
    virtual void setRobot(IRobot&) = 0;
    virtual void setSource(ISource&) = 0;
    virtual state getState() const = 0;
};

}

#endif // ILOOP_H
