#ifndef CLOOPBUILDERBASIC_H
#define CLOOPBUILDERBASIC_H

#include "ILoopBuilder.h"

namespace vs
{

class CLoopBuilderBasic : public ILoopBuilder
{
public:
    CLoopBuilderBasic();
    ILoop build();

private:
    ILoop* m_loop;
    void setController();
    void setFeatureExtractor();
    void setRobot();
    void setSource();
};

}

#endif // CLOOPBUILDERBASIC_H
