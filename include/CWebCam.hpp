#ifndef CWEBCAM_H
#define CWEBCAM_H

namespace vs
{

/**
 * @brief The CWebCam class
 */
class CWebCam : public ISensor
{
public:

    virtual cv::Mat getState() const
    {
        std::cout << "[DummySensor] Getting state " << mState << std::endl;
        return mState;
    }

    virtual void setState(cv::Mat state){mState = state;}

private:
    cv::Mat mState;

};

} // namespace vs

#endif // CWEBCAM_H
