#ifndef IFEATUREEXTRACTOR_H
#define IFEATUREEXTRACTOR_H

#include <vector>

namespace vs
{

class IFeatureExtractor
{
public:
    virtual ~IFeatureExtractor(){}
    virtual std::vector<double> extract() = 0;
};

}

#endif // IFEATUREEXTRACTOR_H
