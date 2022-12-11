#include "../include/CROSFrameGrabber.hpp"

namespace vs
{

/**
 * @brief The CROSFrameGrabber::pImpl class
 */
class CROSFrameGrabber::pImpl
{
    friend class CROSFrameGrabber;

private:
    pImpl(ros::NodeHandle nh) : mImageTransport(nh),
                                mCurrentFrame(new cv_bridge::CvImage)
    {}

    image_transport::ImageTransport mImageTransport;
    cv_bridge::CvImagePtr mCurrentFrame;
    image_transport::Subscriber mImageSubscriber;
    void callback(const sensor_msgs::ImageConstPtr&);
};


/**
 * @brief Default constructor
 */
CROSFrameGrabber::CROSFrameGrabber(ros::NodeHandle nh, std::string& ros_topic)
    : mImpl(new CROSFrameGrabber::pImpl(nh))
{
    mImpl->mImageSubscriber = mImpl->mImageTransport.subscribe(ros_topic.c_str(), 1,
                                                               &CROSFrameGrabber::pImpl::callback,
                                                               mImpl.get());
}


/**
 * @brief CROSFrameGrabber::~CROSFrameGrabber
 */
CROSFrameGrabber::~CROSFrameGrabber(){}


/**
 * @brief Return the current frame
 *
 * @param frame
 */
void
CROSFrameGrabber::getState(cv::Mat& state) const
{
    if(mImpl->mCurrentFrame)
    {
        cv::waitKey(1);
        state = mImpl->mCurrentFrame->image.clone();
    }
}


/**
 * @brief Set image subscriber topic
 *
 * @param topic
 */
void
CROSFrameGrabber::setTopic(std::string ros_topic)
{
    mImpl->mImageSubscriber = mImpl->mImageTransport.subscribe(ros_topic.c_str (), 1,
                                                 &CROSFrameGrabber::pImpl::callback,
                                                 mImpl.get());
}

/**
 * @brief Image subscriber Callback
 *
 * @param msg
 */
void
CROSFrameGrabber::pImpl::callback(const sensor_msgs::ImageConstPtr& msg)
{
    mCurrentFrame = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
}

} // namespace vs
