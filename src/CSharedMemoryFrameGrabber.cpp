#include <string>

#include "../../VS/include/utils.hpp"
#include "../../VS/include/CSharedMemoryFrameGrabber.hpp"

namespace vs
{

/**
 * @brief The CSharedMemoryFrameGrabber::pImpl class
 */
class CSharedMemoryFrameGrabber::pImpl
{
    friend class CSharedMemoryFrameGrabber;

private:

    pImpl(const std::string& sharedObjectName)
    {
        boost::interprocess::managed_shared_memory msm(boost::interprocess::open_only,
                                                       sharedObjectName.c_str());
        this->mSharedImageHeader = msm.find<vs::utils::SharedImageHeader>("MatHeader").first;
        this->mCurrentFrame = cv::Mat(this->mSharedImageHeader->size, this->mSharedImageHeader->type,
                                      msm.get_address_from_handle(this->mSharedImageHeader->handle));
    }

    vs::utils::SharedImageHeader* mSharedImageHeader;
    cv::Mat mCurrentFrame;

};


/**
 * @brief Default constructor
 */
CSharedMemoryFrameGrabber::CSharedMemoryFrameGrabber(const std::string& sharedObjectName)
    : mImpl(new CSharedMemoryFrameGrabber::pImpl(sharedObjectName))
{

}


/**
 * @brief CSharedMemoryFrameGrabber::~CSharedMemoryFrameGrabber
 */
CSharedMemoryFrameGrabber::~CSharedMemoryFrameGrabber(){}


/**
 * @brief Return the current frame
 *
 * @param frame
 */
void
CSharedMemoryFrameGrabber::getState(cv::Mat& state) const
{
    if(!mImpl->mCurrentFrame.empty())
    {
        cv::waitKey(1);
        state = mImpl->mCurrentFrame.clone ();
    }
}


} // namespace vs
