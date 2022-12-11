#include "../../VS/include/CVehicleInterface.hpp"

#define DEBUG

#ifdef DEBUG
#define dbg(msg) std::cout << "[DBG] " << __TIME__ << ":" << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": " << msg << std::endl;
#else
#define dbg(msg)
#endif

namespace vs {

class CVehicleInterface::pImpl
{
    friend class CVehicleInterface;

private:
    pImpl(const ros::NodeHandle& nodeHandle)
        : pose_pub_count(20),
          vel_pub_count(1),
          mPoseIsInitialized(false),
          mTargetInFOV(false),
          mNDOFIsInitialized(false),
          mHomePoseIsSet(false),
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
    bool mHomePoseIsSet;

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

    cv::Mat mHomePose;

    void init();
    void odomCallback(const nav_msgs::Odometry& odometry);
};


/**
 * @brief CArmInterface::pImpl::init
 * @param nodeHandle
 */
void
CVehicleInterface::pImpl::init()
{
    mPosePublisher = mNodeHandle.advertise<nav_msgs::Odometry>("/uwsim/odometry_command",1);
    mVelocityPublisher = mNodeHandle.advertise<nav_msgs::Odometry>("/uwsim/odometry_command",1);
    mPoseSubscriber = mNodeHandle.subscribe("/uwsim/odometry", 1, &CVehicleInterface::pImpl::odomCallback, this);

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
CVehicleInterface::pImpl::odomCallback(const nav_msgs::Odometry& odometry)
{
    if(mNDOFIsInitialized == true)
    {
        dbg("Updating pose")
        mDOFPositions.at<double>(0) = (odometry.pose.pose.position.x);
        mDOFPositions.at<double>(1) = (odometry.pose.pose.position.y);
        mDOFPositions.at<double>(2) = (odometry.pose.pose.position.z);
        mDOFPositions.at<double>(3) = (odometry.pose.pose.orientation.x);
        mDOFPositions.at<double>(4) = (odometry.pose.pose.orientation.y);
        mDOFPositions.at<double>(5) = (odometry.pose.pose.orientation.z);

        mDOFVelocities.at<double>(0) = (odometry.twist.twist.linear.x);
        mDOFVelocities.at<double>(1) = (odometry.twist.twist.linear.y);
        mDOFVelocities.at<double>(2) = (odometry.twist.twist.linear.z);
        mDOFVelocities.at<double>(3) = (odometry.twist.twist.angular.x);
        mDOFVelocities.at<double>(4) = (odometry.twist.twist.angular.y);
        mDOFVelocities.at<double>(5) = (odometry.twist.twist.angular.z);

    }

    mPoseIsInitialized = true;
}


////////// End of pimpl implementation //////////

/**
 * @brief CVehicleInterface::CVehicleInterface
 * @param nodeHandle
 */
CVehicleInterface::CVehicleInterface(const ros::NodeHandle& nodeHandle)
    : mPimpl(new CVehicleInterface::pImpl(nodeHandle))
{}


/**
 * @brief CVehicleInterface::~CVehicleInterface
 */
CVehicleInterface::~CVehicleInterface(){}


/**
 * @brief CVehicleInterface::sendControlSignal
 * @param controlSignal
 */
void
CVehicleInterface::sendControlSignal(cv::Mat &controlSignal)
{
   this->setDOFVelocities(controlSignal);
}


/**
 * @brief
 */
void
CVehicleInterface::manualDrive()
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

        double increment = 0.1;

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

        case 'r':
            // ResetROV pose
            std::cout << "[+] Reseting ROV pose..." << std::endl;
            if(mPimpl->mHomePoseIsSet)
            {
                for(uint i=0; i<mPimpl->mNDOF; ++i)
                {
                    mPimpl->mDOFPositions.at<double>(i) = mPimpl->mHomePose.at<double>(i);
                }
                this->setDOFPositions(mPimpl->mDOFPositions);
            }
            else
            {
                std::cout << "[!] No home pose has been set." << std::endl;
            }
            break;

        case 'i':
            // Set increment
            std::cout << "[+] Set increment:" << std::endl;
            std::cin >> increment;
            break;

        case '8':
            // Translate forward (x += increment)
            mPimpl->mDOFPositions.at<double>(0) += increment;
            this->setDOFPositions(mPimpl->mDOFPositions);
            break;

        case '2':
            // Translate backwards (x -= increment)
            mPimpl->mDOFPositions.at<double>(0) -= increment;
            this->setDOFPositions(mPimpl->mDOFPositions);
            break;

        case '4':
            // Translate left (y -= incremenet)
            mPimpl->mDOFPositions.at<double>(1) -= increment;
            this->setDOFPositions(mPimpl->mDOFPositions);
            break;

        case '6':
            // Translate right (y += increment)
            mPimpl->mDOFPositions.at<double>(1) += increment;
            this->setDOFPositions(mPimpl->mDOFPositions);
            break;

        case '-':
            // Translate downwards (z += increment)
            mPimpl->mDOFPositions.at<double>(2) += increment;
            this->setDOFPositions(mPimpl->mDOFPositions);
            break;

        case '+':
            // Translate upwards (z -= increment)
            mPimpl->mDOFPositions.at<double>(2) -= increment;
            this->setDOFPositions(mPimpl->mDOFPositions);
            break;

        case '1':
            // Roll left
            mPimpl->mDOFPositions.at<double>(3) -= increment;
            this->setDOFPositions(mPimpl->mDOFPositions);
            break;

        case '3':
            // Roll right
            mPimpl->mDOFPositions.at<double>(3) += increment;
            this->setDOFPositions(mPimpl->mDOFPositions);
            break;

        case '/':
            // Pitch backwards
            mPimpl->mDOFPositions.at<double>(4) -= increment;
            this->setDOFPositions(mPimpl->mDOFPositions);
            break;

        case '*':
            // Pitchi forwards
            mPimpl->mDOFPositions.at<double>(4) += increment;
            this->setDOFPositions(mPimpl->mDOFPositions);
            break;

        case '7':
            // Yaw left
            mPimpl->mDOFPositions.at<double>(5) -= increment;
            this->setDOFPositions(mPimpl->mDOFPositions);
            break;

        case '9':
            // Yaw right
            mPimpl->mDOFPositions.at<double>(5) += increment;
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
 * @brief
 *
 * @param ROV_vel_pub
 * @param ROV_v[]
 */
void
CVehicleInterface::setDOFVelocities(const cv::Mat& v)
{
    dbg("")

    nav_msgs::Odometry odom;
    odom.pose.pose.position.x=0.0;
    odom.pose.pose.position.y=0.0;
    odom.pose.pose.position.z=0.0;
    odom.pose.pose.orientation.x=0.0;
    odom.pose.pose.orientation.y=0.0;
    odom.pose.pose.orientation.z=0.0;
    odom.pose.pose.orientation.w=1;

    // Only 4 controllable DOF as tracking 4 lines
    // Need to use X, Y, Z and yaw ;
    odom.twist.twist.linear.x = -v.at<double>(1); // World x = image -y
    odom.twist.twist.linear.y = v.at<double>(0); // World y = image x
    odom.twist.twist.linear.z = v.at<double>(2);
    odom.twist.twist.angular.x = v.at<double>(3);
    odom.twist.twist.angular.y = v.at<double>(4);
    odom.twist.twist.angular.z = v.at<double>(5);
    for(int i=0; i<36; ++i)
    {
        odom.twist.covariance[i]=0;
        odom.pose.covariance[i]=0;
    }
    for(int j=0; j<mPimpl->vel_pub_count; ++j)
    {
        mPimpl->mVelocityPublisher.publish(odom);
        ros::spinOnce();
        mPimpl->mRate.sleep();
    }
} // setVelocity()


/**
 * @brief
 *
 * @param ROV_pos_pub
 * @param ROV_q[]
 */
void
CVehicleInterface::setDOFPositions(const cv::Mat& q)
{
    dbg("")

    nav_msgs::Odometry odom;
    odom.pose.pose.position.x = q.at<double>(0);
    odom.pose.pose.position.y = q.at<double>(1);
    odom.pose.pose.position.z = q.at<double>(2);
    odom.pose.pose.orientation.x = q.at<double>(3);
    odom.pose.pose.orientation.y = q.at<double>(4);
    odom.pose.pose.orientation.z = q.at<double>(5);
    odom.pose.pose.orientation.w=1;

    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = 0.0;

    for(uint i=0; i<36; ++i)
    {
        odom.twist.covariance[i]=0;
        odom.pose.covariance[i]=0;
    }

    for(int i=0; i<mPimpl->pose_pub_count; ++i)
    {
        mPimpl->mPosePublisher.publish(odom);
        ros::spinOnce();
        mPimpl->mRate.sleep();
    }
} // setPose()


/**
  * @brief CVehicleInterface::setHomePose
  * @param homePose
  */
void
CVehicleInterface::setHomePose(const cv::Mat& homePose)
{
    mPimpl->mHomePose = homePose;
    mPimpl->mHomePoseIsSet = true;
}


/**
 * @brief CVehicleInterface::getDOFVelocities
 * @param dofVelocities
 */
void
CVehicleInterface::getDOFVelocities(cv::Mat& dofVelocities)
{
    dofVelocities = mPimpl->mDOFVelocities;
}


/**
 * @brief CVehicleInterface::getDOFPositions
 * @param dofPositions
 */
void
CVehicleInterface::getDOFPositions(cv::Mat& dofPositions)
{
    dofPositions = mPimpl->mDOFPositions;
}



/**
 * @brief CVehicleInterface::setNDOF
 * @param nDOF
 */
void
CVehicleInterface::setNDOF(const uint nDOF)
{
    mPimpl->mNDOF = nDOF;
    mPimpl->mDOFVelocities = cv::Mat::zeros(mPimpl->mNDOF, 1, CV_64F);
    mPimpl->mDOFPositions = cv::Mat::zeros(mPimpl->mNDOF, 1, CV_64F);
}


/**
 * @brief CVehicleInterface::stop
 */
void
CVehicleInterface::stop()
{
    cv::Mat velocities = cv::Mat::zeros(mPimpl->mNDOF, 1, CV_64F);
    this->setDOFVelocities(velocities);
}

} // namespace
