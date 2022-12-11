#ifndef IRUNNER_H
#define IRUNNER_H

#include "vs.h"

namespace vs
{

class IRunner
{
public:
    IRunner();
    virtual ~IRunner(){}
    virtual void start();
    virtual void stop();
    virtual state getState() const;
};

}

#endif // IRUNNER_H
