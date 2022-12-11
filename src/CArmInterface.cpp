#define DEBUG

#ifdef DEBUG
    #define dbg(msg) std::cout << "[DBG] " << __TIME__ << ":" << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": " << msg << std::endl;
#else
    #define dbg(msg)
#endif

#include <sensor_msgs/JointState.h>

#include "../../VS/include/CArmInterface.hpp"
#include "../../VS/include/utils.hpp"

namespace vs
{

/**
 * @brief The CArmInterface::mPimpl class
 */
class CArmInterface::pImpl
{
    friend class CArmInterface;

private:
    pImpl(const ros::NodeHandle& nodeHandle)
        : pose_pub_count(20),
          vel_pub_count(1),
          mPoseIsInitialized(false),
          mTargetInFOV(false),
          mNDOFIsInitialized(false),
          mNodeHandle(nodeHandle),
          mRate(30),
          mIncrement(0.1),
          mDir(1.0)
    {
        this->init();
    }

    int pose_pub_count;
    int vel_pub_count;

    bool mPoseIsInitialized;
    bool mTargetInFOV;
    bool mNDOFIsInitialized;
    ros::NodeHandle mNodeHandle;
    ros::Rate mRate;
    ros::Publisher mVelocityPublisher;
    ros::Publisher mPosePublisher;
    ros::Subscriber mPoseSubscriber;

    double mIncrement;
    double mDir;
    cv::Mat mDOFPositions;
    cv::Mat mDOFVelocities;
    uint mNDOF;

    void init();
    void odomCallback(const sensor_msgs::JointState& jointState);
};


/**
 * @brief CArmInterface::pImpl::init
 * @param nodeHandle
 */
void
CArmInterface::pImpl::init()
{
    mPosePublisher = mNodeHandle.advertise<sensor_msgs::JointState>("/uwsim/joint_state_command",1);
    mVelocityPublisher = mNodeHandle.advertise<sensor_msgs::JointState>("/uwsim/joint_state_command",1);
    mPoseSubscriber = mNodeHandle.subscribe("/uwsim/joint_state", 1, &CArmInterface::pImpl::odomCallback,
                                            this);

    // Get current pose
    while(mPoseIsInitialized == false)
    {
        ros::spinOnce();
    }

}


/**
 * @brief CArmInterface::pImpl::odomCallback
 * @param jointState
 */
void
CArmInterface::pImpl::odomCallback(const sensor_msgs::JointState& jointState)
{
    dbg("")

    if(mNDOFIsInitialized == true)
    {
        for(uint i=0; i<mNDOF; ++i)
        {
            mDOFPositions.at<double>(i) = (jointState.position[i]);
            mDOFVelocities.at<double>(i) = (jointState.velocity[i]);
        }
    }

    mPoseIsInitialized = true;
}

/////////// End of pimpl implementation ///////////

/**
 * @brief CArmInterface::CArmInterface
 * @param nodeHandle
 */
CArmInterface::CArmInterface(const ros::NodeHandle& nodeHandle)
    : mPimpl(new CArmInterface::pImpl(nodeHandle))
{}


/**
 * @brief CArmInterface::~CArmInterface
 */
CArmInterface::~CArmInterface(){}


/**
 * @brief CArmInterface::sendControlSignal
 * @param controlSignal
 */
void
CArmInterface::sendControlSignal(cv::Mat &controlSignal)
{
    this->setDOFVelocities(controlSignal);
}


/**
 * @brief CArmInterface::manualDrive
 */
void
CArmInterface::manualDrive()
{
  dbg("")
  char key = '\0';
  std::cout << "[+] Driving instructions:" << std::endl;
  std::cout << "[+] q = quit" << std::endl;
  std::cout << "[+] s = save current pose" << std::endl;
  std::cout << "[+] r = reset pose" << std::endl;
  std::cout << "[+] p = manually specify pose" << std::endl;
  std::cout << "[+] i = set increment" << std::endl;
  
  while(key != 's')
  {
      // Read key from user
      std::cout << "[+] Press key to move robot:" << std::endl;
      std::cin >> key;

      switch(key)
      {
          case '!':
              std::cout << "[+] Stopping vehicle..." << std::endl;
              this->stop();
              break;

          case 'q':
              std::cout << "[+] Quitting..." << std::endl;
              exit(0);
              break;

          case 's':
              std::cout << "[+] ROV pose Set." << std::endl;
              mPimpl->mTargetInFOV = true;
              break;

          case '.':
              // Invert direction of motion
              mPimpl->mDir *= -1.0;
              std::cout << "dir = " << mPimpl->mDir << std::endl;
              break;

          case 'r':
              // ResetROV pose 
              std::cout << "[+] Reseting ROV pose..." << std::endl;
              mPimpl->mDOFPositions = cv::Mat::zeros(mPimpl->mNDOF, 1, CV_64F);
              this->setDOFPositions(mPimpl->mDOFPositions);
              break;
              
          case 'i':
              // Set increment
              std::cout << "[+] Set increment:" << std::endl;
              std::cin >> mPimpl->mIncrement;
              break;

          case '0':
              // Shoulder pan
          dbg("")
              mPimpl->mDOFPositions.at<double>(0) += mPimpl->mDir * mPimpl->mIncrement;
          dbg("")
              this->setDOFPositions(mPimpl->mDOFPositions);
          dbg("")

          break;
              
          case '1':
              // Shoulder pitch
              mPimpl->mDOFPositions.at<double>(1) += mPimpl->mDir * mPimpl->mIncrement;
              this->setDOFPositions(mPimpl->mDOFPositions);
              break;
              
          case '2':
              // Shoulder roll
              mPimpl->mDOFPositions.at<double>(2) += mPimpl->mDir * mPimpl->mIncrement;
              this->setDOFPositions(mPimpl->mDOFPositions);
              break;
                        
          case '4':
              // Elbow pitch
              mPimpl->mDOFPositions.at<double>(3) += mPimpl->mDir * mPimpl->mIncrement;
              this->setDOFPositions(mPimpl->mDOFPositions);
              break;
              
          case '5':
              // Elbow roll
              mPimpl->mDOFPositions.at<double>(4) += mPimpl->mDir * mPimpl->mIncrement;
              this->setDOFPositions(mPimpl->mDOFPositions);
              break;
                        
          case '7':
              // Wrist pitch
              mPimpl->mDOFPositions.at<double>(5) += mPimpl->mDir * mPimpl->mIncrement;
              this->setDOFPositions(mPimpl->mDOFPositions);
              break;
              
          case '8':
              // Wrist roll
              mPimpl->mDOFPositions.at<double>(6) += mPimpl->mDir * mPimpl->mIncrement;
              this->setDOFPositions(mPimpl->mDOFPositions);
              break;

          case '-':
              // Grasp
              mPimpl->mDOFPositions.at<double>(7) = -0.5; // Middle finger base
              mPimpl->mDOFPositions.at<double>(8) = 0.5; // Middle finger end

              mPimpl->mDOFPositions.at<double>(9) = -0.5; // Right finger base
              mPimpl->mDOFPositions.at<double>(10) = 0.5; // Right finger end

              mPimpl->mDOFPositions.at<double>(11) = -0.5; // Left finger base
              mPimpl->mDOFPositions.at<double>(12) = 0.5; // Left finger end

              this->setDOFPositions(mPimpl->mDOFPositions);
              break;

         case '+':
              // Release ('un-grasp')
              mPimpl->mDOFPositions.at<double>(7) = 0.0; // Middle finger base
              mPimpl->mDOFPositions.at<double>(8) = 0.0; // Middle finger end

              mPimpl->mDOFPositions.at<double>(9) = 0.0; // Right finger base
              mPimpl->mDOFPositions.at<double>(10) = 0.0; // Right finger end

              mPimpl->mDOFPositions.at<double>(11) = 0.0; // Left finger base
              mPimpl->mDOFPositions.at<double>(12) = 0.0; // Left finger end

              this->setDOFPositions(mPimpl->mDOFPositions);
              break;

         case 'z':
              // 'z' config
              mPimpl->mDOFPositions.at<double>(0) = 0.0;
              mPimpl->mDOFPositions.at<double>(1) = -1.2;
              mPimpl->mDOFPositions.at<double>(2) = 0.0;
              
              mPimpl->mDOFPositions.at<double>(3) = 2.5;
              mPimpl->mDOFPositions.at<double>(4) = -1.57;
              
              mPimpl->mDOFPositions.at<double>(5) = -1.3;
              mPimpl->mDOFPositions.at<double>(6) = 1.57;

              this->setDOFPositions(mPimpl->mDOFPositions);
              break;

          default:
              std::cout << "[+] Valid options are:" << std::endl;
              std::cout << "\ti, p, q, r, s, v, 1-9, /, *, -, +" << std::endl;
              break;
      } // switch()
      
      ros::spinOnce();
      
  } // while()

}


/**
 * @brief CArmInterface::setDOFVelocities
 * @param velocities
 */
void
CArmInterface::setDOFVelocities(const cv::Mat& velocities)
{
    sensor_msgs::JointState js;

    for(int i=0; i<velocities.rows; ++i)
    {
        js.velocity.push_back(velocities.at<double>(i));
    }

    for(int j=0; j<mPimpl->vel_pub_count; ++j)
    {
        mPimpl->mVelocityPublisher.publish(js);
        ros::spinOnce();
        mPimpl->mRate.sleep();
    }
} // setVelocity()


/**
 * @brief CArmInterface::setDOFPositions
 * @param positions
 */
void
CArmInterface::setDOFPositions(const cv::Mat& positions)
{
    sensor_msgs::JointState js;
    
    for(int i=0; i<positions.rows; ++i)
    {
        js.position.push_back(positions.at<double>(i));
    }

    for(int i=0; i<mPimpl->pose_pub_count; ++i)
    {
       mPimpl->mPosePublisher.publish(js);
        ros::spinOnce();
        mPimpl->mRate.sleep();
    }
} // setDOFPositions()


/**
 * @brief CArmInterface::setNDOF
 * @param nDOF
 */
void
CArmInterface::setNDOF(const uint nDOF)
{
    mPimpl->mNDOF = nDOF;
    mPimpl->mDOFVelocities = cv::Mat::zeros(mPimpl->mNDOF, 1, CV_64F);
    mPimpl->mDOFPositions = cv::Mat::zeros(mPimpl->mNDOF, 1, CV_64F);
}


/**
 * @brief CArmInterface::stop
 */
void
CArmInterface::stop()
{
    cv::Mat velocities = cv::Mat::zeros(mPimpl->mNDOF, 1, CV_64F);
    this->setDOFVelocities(velocities);
}


/**
 * @brief CArmInterface::getDOFVelocities
 * @param dofVelocities
 */
void
CArmInterface::getDOFVelocities(cv::Mat& dofVelocities)
{
    dofVelocities = mPimpl->mDOFVelocities;
}


/**
 * @brief CArmInterface::getDOFPositions
 * @param dofPositions
 */
void
CArmInterface::getDOFPositions(cv::Mat& dofPositions)
{
    dofPositions = mPimpl->mDOFPositions;
}

} // namespace

