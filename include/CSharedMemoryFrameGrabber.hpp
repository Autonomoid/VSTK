#ifndef CSHAREDMEMORYFRAMEGRABBER_HPP
#define CSHAREDMEMORYFRAMEGRABBER_HPP

#include <boost/scoped_ptr.hpp>

#include "IVisionSensor.hpp"

namespace vs
{

/**
 * @brief Class to grab image frames from shared memory.
 */
class CSharedMemoryFrameGrabber : public IVisionSensor
{
  public:
    CSharedMemoryFrameGrabber(const std::string& sharedObjectName);
    ~CSharedMemoryFrameGrabber();
    void setSharedObjectName(const std::string& sharedObjectName);
    void getState(cv::Mat& state) const;

  private:
    class pImpl;
    boost::scoped_ptr<pImpl> mImpl;
};

} // namespace vs


#endif // CSHAREDMEMORYFRAMEGRABBER_HPP
