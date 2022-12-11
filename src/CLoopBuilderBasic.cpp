#include "../include/CLoopBuilderBasic.h"
#include "../include/CLoopBasic.h"

#include "../include/CControllerBasic.h"
#include "../include/CFeatureExtractorBasic.h"
#include "../include/CRobotBasic.h"
#include "../include/CSourceBasic.h"

CLoopBuilderBasic::CLoopBuilderBasic() : m_loop(new CLoopBasic)
{}

ILoop
CLoopBuilderBasic::build()
{
    this->setController();
    this->setFeatureExtractor();
    this->setRobot();
    this->setSource();
    return this->m_loop;
}

void
CLoopBuilderBasic::setController()
{
    this->m_loop->setController(new CControllerBasic);
}

void
CLoopBuilderBasic::setFeatureExtractor()
{
    this->m_loop->setFeatureExtractor(new CFeatureExtractorBasic);
}

void
CLoopBuilderBasic::setRobot()
{
    this->m_loop->setRobot(new CRobotBasic);
}

void
CLoopBuilderBasic::setSource()
{
    this->m_loop->setSource(new CSourceBasic);
}
