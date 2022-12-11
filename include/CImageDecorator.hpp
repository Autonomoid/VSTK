#ifndef CIMAGEDECORATOR_H
#define CIMAGEDECORATOR_H

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class CImageDecorator
{
    public:
        CImageDecorator();
        ~CImageDecorator();
        void setImage(cv::Mat);
        cv::Mat getImage();
        void display();
        void quiet();
        void addPoint(cv::Point2f p, std::string label);
        void addPoints(std::vector<cv::Point2f>, bool);
        void addText(std::string, cv::Point2f);    
        void setColour(cv::Scalar);
        cv::Scalar getJetColor(double value) const;
        
    private:
        bool QUIET;
        cv::Mat image;
        cv::Scalar colour;
        
};

#endif
