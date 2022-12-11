#ifndef ISOURCE_H
#define ISOURCE_H

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace vs
{

class ISource
{
public:
    virtual ~ISource(){}
    virtual cv::Mat getData() const = 0;

};

#endif // ISOURCE_H
