#ifndef ILOOPBUILDER_H
#define ILOOPBUILDER_H

#include "ILoop.h"

namespace vs
{

class ILoopBuilder
{
public:
    virtual ~ILoopBuilder(){}
    virtual ILoop build() = 0;

private:
    virtual void setController() = 0;
    virtual void setFeatureExtractor() = 0;
    virtual void setRobot() = 0;
    virtual void setSource() = 0;
};

}

#endif // ILOOPBUILDER_H
