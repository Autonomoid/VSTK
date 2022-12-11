#ifndef VS_H
#define VS_H

/*#include "CControllerBasic.h"
#include "CFeatureExtractorBasic.h"
#include "CLoopBasic.h"
#include "CLoopBuilderBasic.h"
#include "CRobotBasic.h"
#include "CRunnerBasic.h"
#include "CSourceBasic.h"*/

namespace vs
{

    typedef enum state
    {
        IDLE,
        RUNNING,
        STOPPED,
        ERROR,
        CONVERGED,
        STALLED
    } state;

}

#endif // VS_H
