#ifndef CSOURCEBASIC_H
#define CSOURCEBASIC_H

#include "../include/ISource.h"

namespace vs
{

class CSourceBasic : public ISource
{
public:
    CSourceBasic();
    cv::Mat getData() const;

private:
    cv::Mat data;
};

}

#endif // CSOURCEBASIC_H
