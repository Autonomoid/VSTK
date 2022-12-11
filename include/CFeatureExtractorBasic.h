#ifndef CFEATUREEXTRACTORBASIC_H
#define CFEATUREEXTRACTORBASIC_H

#include "IFeatureExtractor.h"

namespace vs
{

class CFeatureExtractorBasic : public IFeatureExtractor
{
public:
    CFeatureExtractorBasic();
    std::vector<double> extract();
};

}

#endif // CFEATUREEXTRACTORBASIC_H
