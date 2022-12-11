#include <math.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "../include/CLink.hpp"

    int CLink::nextID = 0;

    /**
     * @brief 
     *
     * @param d
     * @param r
     * @param alpha
     * @param theta
     * @param offset
     * @param real
     */
    CLink::CLink(double d, double r, double alpha, double theta, double offset, double real)
    {
        this->DH_transform = cv::Mat::eye(4, 4, CV_64F);  // 4x4 matrix
        this->ID = nextID;
        nextID += 1;
        this->DH_d = d;
        this->DH_r = r;
        this->DH_alpha = alpha;
        this->DH_theta = theta;
        this->offset = offset;
    }

    //CLink::~CLink(){}

    /**
     * @brief 
     *
     * @return 
     */
    int
    CLink::getID() const
    {
        return this->ID;
    }

    /**
     * @brief 
     *
     * @return 
     */
    double
    CLink::getDH_d() const
    {
        return this->DH_d;
    }

    /**
     * @brief 
     *
     * @return 
     */
    double
    CLink::getDH_r() const
    {
        return this->DH_r;
    }

    /**
     * @brief 
     *
     * @return 
     */
    double
    CLink::getDH_alpha() const
    {
        return this->DH_alpha;
    }

    /**
     * @brief 
     *
     * @return 
     */
    double
    CLink::getDH_theta() const
    {
        return this->DH_theta;
    }

    /**
     * @brief 
     *
     * @return 
     */
    double
    CLink::getOffset() const
    {
        return this->offset;
    }

    /**
     * @brief 
     *
     * @return 
     */
    double
    CLink::getReal() const
    {
        return this->real;
    }

    /**
     * @brief 
     *
     * @param theta
     *
     * @return 
     */
    cv::Mat
    CLink::getTransform(double theta)
    {
        // Compute Denevit-Haternberg transformation.
        double sa = sin(this->DH_alpha);
        double ca = cos(this->DH_alpha);
        double st = sin(theta - this->real + this->offset);
        double ct = cos(theta - this->real + this->offset);

        this->DH_transform.at<double>(0, 0) = ct;
        this->DH_transform.at<double>(0, 1) = -ca*st;
        this->DH_transform.at<double>(0, 2) = sa*st;
        this->DH_transform.at<double>(0, 3) = DH_r*ct;

        this->DH_transform.at<double>(1, 0) = st;
        this->DH_transform.at<double>(1, 1) = ca*ct;
        this->DH_transform.at<double>(1, 2) = -sa*ct;
        this->DH_transform.at<double>(1, 3) = DH_r*st; 

        this->DH_transform.at<double>(2, 0) = 0;
        this->DH_transform.at<double>(2, 1) = sa;
        this->DH_transform.at<double>(2, 2) = ca;
        this->DH_transform.at<double>(2, 3) = DH_d; //

        this->DH_transform.at<double>(3, 0) = 0;
        this->DH_transform.at<double>(3, 1) = 0;
        this->DH_transform.at<double>(3, 2) = 0;
        this->DH_transform.at<double>(3, 3) = 1;
        return this->DH_transform;
    }

