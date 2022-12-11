//#define DEBUG

#ifdef DEBUG
    #define dbg(msg) std::cout << "[DBG] " << __TIME__ << ":" << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": " << msg << std::endl;
#else
    #define dbg(msg)
#endif

#include <sensor_msgs/JointState.h>

#include "../../VS/include/CArmModel.hpp"
#include "../../VS/include/utils.hpp"

namespace vs
{

/**
 * @brief The pImpl class
 */
class CArmModel::pImpl
{
    friend class CArmModel;

private:
    pImpl(): mNDOF(1), mNDOFIsInitialized(false){}

    void getPoseFromJointAngles(const cv::Mat& q, cv::Mat& pose) const;
    void getPoseFromJointAngles(cv::Mat& pose) const;

    void getJointAnglesFromPose(const cv::Mat& pose, cv::Mat& q) const;

    void getRobotJacobian(const cv::Mat& jointAngles, cv::Mat& jacobian) const;
    void getRobotJacobian(cv::Mat& jacobian) const;

    void get_wXe(cv::Mat& wXe) const;
    void get_wMe(cv::Mat& wMe) const;
    void get_cMe(cv::Mat& cMe) const;
    void get_eJe(const cv::Mat& dofPositions, cv::Mat& eJe) const;
    void get_cJe(cv::Mat& cJe) const;
    void get_cJe(const cv::Mat& dofPositions, cv::Mat& cJe) const;
    void get_fMe(cv::Mat& wMe) const;

    void updateJointAngles(const sensor_msgs::JointState& currentJointAngles);

    void getJointVelocityFromCameraVelocity(const cv::Mat& cVc, cv::Mat& q_dot) const;

    cv::Mat mCurrentJointAngles;

    uint mNDOF;
    bool mNDOFIsInitialized;

    ros::NodeHandle mNodeHandle;
    ros::Subscriber mJointAngleSubscriber;
    ros::Subscriber mPoseSubscriber;
};


/**
 * @brief CArmModel::pImpl::updateJointAngles
 * @param currentJointAngles
 */
void
CArmModel::pImpl::updateJointAngles(const sensor_msgs::JointState& currentJointAngles)
{
  if(mNDOFIsInitialized == true)
  {
    for(uint i=0; i<mNDOF; ++i)
    {
      mCurrentJointAngles.at<double>(i) = currentJointAngles.position[i];
    }
  }
}


/**
 * @brief CArmModel::pImpl::getJointAnglesFromPose
 * @param desiredWristPose
 * @param jointAngles
 */
void
CArmModel::pImpl::getJointAnglesFromPose(const cv::Mat& desiredWristPose, cv::Mat& q) const
{
  // Sanity check
  if(mCurrentJointAngles.rows < 1)
  {
    dbg("ERROR -- No joint angles!")
    throw "ERROR -- No joint angles!";
  }

  std::vector<double> q_solution; // Temporary joint angles / solution.
  std::vector<double> jointAngles;
  for(uint i=0; i<7; ++i)
  {
    jointAngles.push_back(mCurrentJointAngles.at<double>(i));
  }

  std::vector<double> delta_vec;
  cv::Mat currentWristPose = cv::Mat::zeros(4, 4, CV_64F); // Temporary wrist-to-base transform.
  currentWristPose.at<double>(3,3) = 1.0; // Add the homogeneous co-ordinate

  cv::Mat virtualCurrentPose(6, 1, CV_64F);
  cv::Mat virtualDesiredpose(6, 1, CV_64F);

  cv::Mat jacobian(6, 7, CV_64F);
  cv::Mat inverseJacobian(7, 6, CV_64F);
  double lambda = 0.1;

  uint maxIter = 100;

  // Gradient descent routine
  for(uint i=0; i<maxIter; ++i)
  {
      this->getPoseFromJointAngles(cv::Mat(jointAngles), currentWristPose);
      this->getRobotJacobian(cv::Mat(jointAngles), jacobian);

      // Convert transforms into pose vectors.
      vs::utils::HTransformToPose(currentWristPose, virtualCurrentPose);
      vs::utils::HTransformToPose(desiredWristPose, virtualDesiredpose);

      // Update joint positions
      cv::invert(jacobian, inverseJacobian, cv::DECOMP_SVD);
      cv::Mat delta_mat = -lambda * inverseJacobian * (virtualCurrentPose - virtualDesiredpose);
      delta_mat.col(0).copyTo(delta_vec);

      // Update q_solution witht he incremental change delta_vec
      std::vector<double>::iterator it_1 = q_solution.begin();
      std::vector<double>::iterator it_2 = delta_vec.begin();
      for(; it_1!=q_solution.end(), it_2!=delta_vec.end(); ++it_1, ++it_2)
      {
          *it_1 = *it_1 + *it_2;
      }
  }

  q = cv::Mat(jointAngles);

} // getJointAnglesFromPose()


/**
 * @brief CArmModel::pImpl::getPoseFromJointAngles
 * @param pose
 */
void
CArmModel::pImpl::getPoseFromJointAngles(cv::Mat& pose) const
{
    this->getPoseFromJointAngles(this->mCurrentJointAngles, pose);
}


/**
 * @brief CArmModel::pImpl::getPoseFromJointAngles
 * @param jointAngles
 * @param pose
 */
void
CArmModel::pImpl::getPoseFromJointAngles(const cv::Mat& jointAngles, cv::Mat& pose) const
{
  // Sanity check.
  if(jointAngles.rows < mNDOF)
  {
    dbg("ERROR - not enough joint angles")
    throw "ERROR - not enough joint angles";
  }

  // Is the pose matrix a 4x4 (homogeneous) matrix?
  if(pose.rows!=4 || pose.cols!=4)
  {
    dbg("ERROR - pose matrix wrong dimensions")
    pose = cv::Mat(4, 4, CV_64F);
  }

  const double c1 = cos(jointAngles.at<double>(0));
  const double c2 = cos(jointAngles.at<double>(1));
  const double c3 = cos(jointAngles.at<double>(2));
  const double c4 = cos(jointAngles.at<double>(3));
  const double c5 = cos(jointAngles.at<double>(4));
  const double c6 = cos(jointAngles.at<double>(5));
  const double c7 = cos(jointAngles.at<double>(6));

  const double s1 = sin(jointAngles.at<double>(0));
  const double s2 = sin(jointAngles.at<double>(1));
  const double s3 = sin(jointAngles.at<double>(2));
  const double s4 = sin(jointAngles.at<double>(3));
  const double s5 = sin(jointAngles.at<double>(4));
  const double s6 = sin(jointAngles.at<double>(5));
  const double s7 = sin(jointAngles.at<double>(6));

  const double r3 = 0.427;
  const double r5 = 0.42;

  // compute Geometrical Model for Wrist frame
  pose.at<double>(0,0) = ((-c1*c2*s3-s1*c3)*s5+((c1*c2*c3-s1*s3)*c4-c1*s2*s4)*c5)*s7+(((c1*c2*c3-s1*s3)*s4+c1*s2*c4)*s6+(((c1*c2*c3-s1*s3)*c4-c1*s2*s4)*s5+(c1*c2*s3+s1*c3)*c5)*c6)*c7;
  pose.at<double>(1,0) = ((c1*c3-s1*c2*s3)*s5+((c1*s3+s1*c2*c3)*c4-s1*s2*s4)*c5)*s7+(((c1*s3+s1*c2*c3)*s4+s1*s2*c4)*s6+(((c1*s3+s1*c2*c3)*c4-s1*s2*s4)*s5+(s1*c2*s3-c1*c3)*c5)*c6)*c7;
  pose.at<double>(2,0) = (s2*s3*s5+(-c2*s4-s2*c3*c4)*c5)*s7+((c2*c4-s2*c3*s4)*s6+((-c2*s4-s2*c3*c4)*s5-s2*s3*c5)*c6)*c7;

  pose.at<double>(0,1) = (((s1*s3-c1*c2*c3)*s4-c1*s2*c4)*s6+((c1*s2*s4+(s1*s3-c1*c2*c3)*c4)*s5+(-c1*c2*s3-s1*c3)*c5)*c6)*s7+((-c1*c2*s3-s1*c3)*s5+((c1*c2*c3-s1*s3)*c4-c1*s2*s4)*c5)*c7;
  pose.at<double>(1,1) = (((-c1*s3-s1*c2*c3)*s4-s1*s2*c4)*s6+((s1*s2*s4+(-c1*s3-s1*c2*c3)*c4)*s5+(c1*c3-s1*c2*s3)*c5)*c6)*s7+((c1*c3-s1*c2*s3)*s5+((c1*s3+s1*c2*c3)*c4-s1*s2*s4)*c5)*c7;
  pose.at<double>(2,1) = ((s2*c3*s4-c2*c4)*s6+((c2*s4+s2*c3*c4)*s5+s2*s3*c5)*c6)*s7+(s2*s3*s5+(-c2*s4-s2*c3*c4)*c5)*c7;

  pose.at<double>(0,2) = ((c1*s2*s4+(s1*s3-c1*c2*c3)*c4)*s5+(-c1*c2*s3-s1*c3)*c5)*s6+((c1*c2*c3-s1*s3)*s4+c1*s2*c4)*c6;
  pose.at<double>(1,2) = ((s1*s2*s4+(-c1*s3-s1*c2*c3)*c4)*s5+(c1*c3-s1*c2*s3)*c5)*s6+((c1*s3+s1*c2*c3)*s4+s1*s2*c4)*c6;
  pose.at<double>(2,2) = ((c2*s4+s2*c3*c4)*s5+s2*s3*c5)*s6+(c2*c4-s2*c3*s4)*c6;

  pose.at<double>(0,3) = (r5*c1*c2*c3-r5*s1*s3)*s4+r5*c1*s2*c4+r3*c1*s2;
  pose.at<double>(1,3) = (r5*c1*s3+r5*s1*c2*c3)*s4+r5*s1*s2*c4+r3*s1*s2;
  pose.at<double>(2,3) = -r5*s2*c3*s4+r5*c2*c4+r3*c2;

  pose.at<double>(3,3) = 1.0;

}


void
CArmModel::pImpl::getRobotJacobian(cv::Mat& jacobian) const
{
    getRobotJacobian(mCurrentJointAngles, jacobian);
}

/**
 * @brief CArmModel::pImpl::getRobotJacobian
 * @param jointAngles
 * @param jacobian
 */
void
CArmModel::pImpl::getRobotJacobian(const cv::Mat& jointAngles, cv::Mat& jacobian) const
{
  // Sanity check
  if(jacobian.rows!=6 || jacobian.cols!=7)
  {
    dbg("ERROR - Jacobian matrix has incorrect dimensions.")
    throw "ERROR - Jacobian matrix has incorrect dimensions.";
  }

    const double c1 =  cos(jointAngles.at<double>(0));
    const double c2 =  cos(jointAngles.at<double>(1));
    const double c3 =  cos(jointAngles.at<double>(2));
    const double c4 =  cos(jointAngles.at<double>(3));
    const double c5 =  cos(jointAngles.at<double>(4));
    const double c6 =  cos(jointAngles.at<double>(5));

    const double s1 =  sin(jointAngles.at<double>(0));
    const double s2 =  sin(jointAngles.at<double>(1));
    const double s3 =  sin(jointAngles.at<double>(2));
    const double s4 =  sin(jointAngles.at<double>(3));
    const double s5 =  sin(jointAngles.at<double>(4));
    const double s6 =  sin(jointAngles.at<double>(5));

    const double r3 = 0.427;
    const double r5 = 0.42;

    // Column 0
    jacobian.at<double>(0,0) = r5*(-(c1*s3+s1*c2*c3)*s4-s1*s2*c4)-r3*s1*s2;
    jacobian.at<double>(1,0) = r3*c1*s2-r5*(-(c1*c2*c3-s1*s3)*s4-c1*s2*c4);
    jacobian.at<double>(2,0) = 0;
    jacobian.at<double>(3,0) = 0;
    jacobian.at<double>(4,0) = 0;
    jacobian.at<double>(5,0) = 1;

    // Column 1
    jacobian.at<double>(0,1) = c1*(r3*c2-r5*(s2*c3*s4-c2*c4));
    jacobian.at<double>(1,1) = s1*(r3*c2-r5*(s2*c3*s4-c2*c4));
    jacobian.at<double>(2,1) = -c1*(r3*c1*s2-r5*(-(c1*c2*c3-s1*s3)*s4-c1*s2*c4))-s1*(r3*s1*s2-r5*(-(c1*s3+s1*c2*c3)*s4-s1*s2*c4));
    jacobian.at<double>(3,1) = -s1;
    jacobian.at<double>(4,1) = c1;
    jacobian.at<double>(5,1) = 0;

    // Column 2
    jacobian.at<double>(0,2) = s1*s2*(r3*c2-r5*(s2*c3*s4-c2*c4))-c2*(r3*s1*s2-r5*(-(c1*s3+s1*c2*c3)*s4-s1*s2*c4));
    jacobian.at<double>(1,2) = c2*(r3*c1*s2-r5*(-(c1*c2*c3-s1*s3)*s4-c1*s2*c4))-c1*s2*(r3*c2-r5*(s2*c3*s4-c2*c4));
    jacobian.at<double>(2,2) = c1*s2*(r3*s1*s2-r5*(-(c1*s3+s1*c2*c3)*s4-s1*s2*c4))-s1*s2*(r3*c1*s2-r5*(-(c1*c2*c3-s1*s3)*s4-c1*s2*c4));
    jacobian.at<double>(3,2) = c1*s2;
    jacobian.at<double>(4,2) = s1*s2;
    jacobian.at<double>(5,2) = c2;

    // Column 3
    jacobian.at<double>(0,3) = r5*s2*s3*(-(c1*s3+s1*c2*c3)*s4-s1*s2*c4)-r5*(c1*c3-s1*c2*s3)*(s2*c3*s4-c2*c4);
    jacobian.at<double>(1,3) = -r5*s2*s3*(-(c1*c2*c3-s1*s3)*s4-c1*s2*c4)-r5*(c1*c2*s3+s1*c3)*(s2*c3*s4-c2*c4);
    jacobian.at<double>(2,3) = -r5*(s1*c2*s3-c1*c3)*(-(c1*c2*c3-s1*s3)*s4-c1*s2*c4)-r5*(-c1*c2*s3-s1*c3)*(-(c1*s3+s1*c2*c3)*s4-s1*s2*c4);
    jacobian.at<double>(3,3) = -c1*c2*s3-s1*c3;
    jacobian.at<double>(4,3) = c1*c3-s1*c2*s3;
    jacobian.at<double>(5,3) = s2*s3;

    // Column 4
    jacobian.at<double>(0,4) = -r5*(s2*c3*s4-c2*c4)*((c1*s3+s1*c2*c3)*s4+s1*s2*c4)-r5*(s2*c3*s4-c2*c4)*(-(c1*s3+s1*c2*c3)*s4-s1*s2*c4);
    jacobian.at<double>(1,4) = -r5*(s2*c3*s4-c2*c4)*(-(c1*c2*c3-s1*s3)*s4-c1*s2*c4)-r5*(c2*c4-s2*c3*s4)*(-(c1*c2*c3-s1*s3)*s4-c1*s2*c4);
    jacobian.at<double>(2,4) = -r5*(-(c1*s3+s1*c2*c3)*s4-s1*s2*c4)*((c1*c2*c3-s1*s3)*s4+c1*s2*c4)-r5*(-(c1*s3+s1*c2*c3)*s4-s1*s2*c4)*(-(c1*c2*c3-s1*s3)*s4-c1*s2*c4);
    jacobian.at<double>(3,4) = (c1*c2*c3-s1*s3)*s4+c1*s2*c4;
    jacobian.at<double>(4,4) = (c1*s3+s1*c2*c3)*s4+s1*s2*c4;
    jacobian.at<double>(5,4) = c2*c4-s2*c3*s4;

    // Column 5
    jacobian.at<double>(0,5) = 0;
    jacobian.at<double>(1,5) = 0;
    jacobian.at<double>(2,5) = 0;
    jacobian.at<double>(3,5) = -(-c1*c2*s3-s1*c3)*s5-((c1*c2*c3-s1*s3)*c4-c1*s2*s4)*c5;
    jacobian.at<double>(4,5) = -(c1*c3-s1*c2*s3)*s5-((c1*s3+s1*c2*c3)*c4-s1*s2*s4)*c5;
    jacobian.at<double>(5,5) = -s2*s3*s5-(-c2*s4-s2*c3*c4)*c5;

    // Column 6
    jacobian.at<double>(0,6) = 0;
    jacobian.at<double>(1,6) = 0;
    jacobian.at<double>(2,6) = 0;
    jacobian.at<double>(3,6) = ((c1*c2*c3-s1*s3)*s4+c1*s2*c4)*c6-(((c1*c2*c3-s1*s3)*c4-c1*s2*s4)*s5-(-c1*c2*s3-s1*c3)*c5)*s6;
    jacobian.at<double>(4,6) = ((c1*s3+s1*c2*c3)*s4+s1*s2*c4)*c6-(((c1*s3+s1*c2*c3)*c4-s1*s2*s4)*s5-(c1*c3-s1*c2*s3)*c5)*s6;
    jacobian.at<double>(5,6) = 1;
}


/**
 * @brief Define the wrist to end-effector pose (wXe).
 * @param wMe
 */
void
CArmModel::pImpl::get_wXe(cv::Mat& wXe) const
{
    wXe = cv::Mat::zeros(6, 1, CV_64F);

    wXe.at<double>(0, 0) = 0.0;
    wXe.at<double>(1, 0) = 0.0;
    wXe.at<double>(2, 0) = 0.153;

    wXe.at<double>(3, 0) = 0.0;
    wXe.at<double>(4, 0) = 0.0;
    wXe.at<double>(5, 0) = 0.0;
}


/**
 * @brief CArmModell::pImpl::get_wMe
 * @param wMe
 */
void
CArmModel::pImpl::get_wMe(cv::Mat& wMe) const
{
    wMe = cv::Mat::zeros(4, 4, CV_64F);
    cv::Mat wXe;
    this->get_wXe(wXe);
    vs::utils::PoseToHTransform(wXe, wMe);
}


/**
 * @brief Define the camera to end-effector transform (cMe).
 * @param cMe
 */
void
CArmModel::pImpl::get_cMe(cv::Mat& cMe) const
{
    cMe = cv::Mat::zeros(6, 1, CV_64F);

    cMe.at<double>(0, 0) = 0.0;
    cMe.at<double>(1, 0) = 0.0;
    cMe.at<double>(2, 0) = -0.2;

    cMe.at<double>(3, 0) = 0.0;
    cMe.at<double>(4, 0) = 0.0;
    cMe.at<double>(5, 0) = -M_PI/2.0;
}


/**
 * @brief CArmModel::pImpl::get_fMe
 * @param fMe
 */
void
CArmModel::pImpl::get_fMe(cv::Mat& fMe) const
{
    cv::Mat fMw = cv::Mat::zeros(4, 4, CV_64F);
    this->getPoseFromJointAngles(fMw);

    cv::Mat wMe;
    this->get_wMe(wMe);

    fMe = fMw * wMe;
}


/**
 * @brief CArmModel::pImpl::get_eJe
 * @param eJe
 */
void
CArmModel::pImpl::get_eJe(const cv::Mat& jointAngles, cv::Mat& eJe) const
{
    cv::Mat fMw = cv::Mat::zeros(4, 4, CV_64F);
    this->getPoseFromJointAngles(jointAngles, fMw);

    cv::Mat wXe;
    this->get_wXe(wXe);

    // Compute the end-effector to wrist transform (eMw).
    cv::Mat eXw = -1.0 * wXe;

    // Construct the skew-symmetric form of
    // the translationsal part of eMw (eTw_x)
    cv::Mat eTw_x = cv::Mat::zeros(3, 3, CV_64F);
    eTw_x.at<double>(0, 0) = 0.0;
    eTw_x.at<double>(0, 1) = -eXw.at<double>(2,0);
    eTw_x.at<double>(0, 2) = eXw.at<double>(1);;

    eTw_x.at<double>(1, 0) = eXw.at<double>(2,0);;
    eTw_x.at<double>(1, 1) = 0.0;
    eTw_x.at<double>(1, 2) = -eXw.at<double>(0);;

    eTw_x.at<double>(2, 0) = -eXw.at<double>(1);;
    eTw_x.at<double>(2, 1) = eXw.at<double>(0);;
    eTw_x.at<double>(2, 2) = 0.0;

    // Extract the fixed to wrist rotation (fRw).
    cv::Mat fRw = cv::Mat(3, 3, CV_64F);
    vs::utils::HTransformToRotationMatrix(fMw, fRw);

    // Compute the inverse (wRf)
    cv::Mat wRf = fRw.inv(cv::DECOMP_SVD);

    // Construct the end-effector to fixed twist (eWf).
    cv::Mat a = eTw_x * wRf;
    cv::Mat tempA;
    vs::utils::hconcat(wRf, a, tempA);

    cv::Mat null_3x3 = cv::Mat::zeros(3, 3, CV_64F);
    cv::Mat tempB;
    vs::utils::hconcat(null_3x3, wRf, tempB);

    cv::Mat eWf;
    vs::utils::vconcat(tempA, tempB, eWf);

    // Transform the Jacobian into the end-effector frame.
    cv::Mat fJw = cv::Mat::zeros(6, mNDOF, CV_64F);;
    this->getRobotJacobian(fJw);

    eJe = eWf * fJw;
}


/**
 * @brief CArmModel::pImpl::get_cJe
 * @param cJe
 */
void
CArmModel::pImpl::get_cJe(cv::Mat& cJe) const
{
    // Compute the corresponding camera to end-effector twist (cWe).
    cv::Mat cMe;
    this->get_cMe(cMe);
    cv::Mat cWe = cv::Mat::eye(6, 6, CV_64F);
    vs::utils::PoseToTwist(cMe, cWe);

    // Get the robot Jacobian in its own frame.
    cv::Mat eJe;
    this->get_eJe(this->mCurrentJointAngles, eJe);

    // Transform it to the camera frame.
    cJe = cWe * eJe;
}


/**
 * @brief CArmModel::pImpl::get_cJe
 * @param dofPositions
 * @param cJe
 */
void
CArmModel::pImpl::get_cJe(const cv::Mat& dofPositions, cv::Mat& cJe) const
{
    // Compute the corresponding camera to end-effector twist (cWe).
    cv::Mat cMe;
    this->get_cMe(cMe);
    cv::Mat cWe = cv::Mat::eye(6, 6, CV_64F);
    vs::utils::PoseToTwist(cMe, cWe);

    // Get the robot Jacobian in its own frame.
    cv::Mat eJe;
    this->get_eJe(dofPositions, eJe);

    // Transform it to the camera frame.
    cJe = cWe * eJe;
}


/**
 * @brief CArmModel::pImpl::getJointVelocityFromCameraVelocity
 * @param cVc
 * @param q_dot
 */
void
CArmModel::pImpl::getJointVelocityFromCameraVelocity(const cv::Mat& cVc, cv::Mat& q_dot) const
{
    cv::Mat cJe;
    this->get_cJe(cJe);

    // Compute it's inverse (eJe_inv)
    cv::Mat cJe_inv;
    cv::invert(cJe, cJe_inv, cv::DECOMP_SVD);

    // Compute the joint velocities (q_dot) that give the
    // corresponding velocity of the camera (cVc).
    q_dot = cJe_inv * cVc;
}

////////// End of pImpl implementation //////////

/**
 * @brief CArmModel::CArmModel
 * @param nodeHandle
 */
CArmModel::CArmModel(const ros::NodeHandle& nodeHandle)
: mImpl(new CArmModel::pImpl)
{
    mImpl->mNodeHandle = nodeHandle;
    mImpl->mJointAngleSubscriber = mImpl->mNodeHandle.subscribe("/uwsim/joint_state", 1,
                                   &CArmModel::pImpl::updateJointAngles, mImpl.get());
}


/**
 * @brief CArmModel::~CArmModel
 */
CArmModel::~CArmModel(){}


/**
 * @brief CArmModel::setNDOF
 * @param n
 */
void
CArmModel::setNDOF(const uint n)
{
  mImpl->mNDOF = n;

  for(uint i=0; i<n; ++i)
  {
      mImpl->mCurrentJointAngles.push_back(0.0);
  }

  mImpl->mNDOFIsInitialized = true;
}


/**
 * @brief CArmModel::getForwardKinematics
 * @param q
 * @param pose
 */
void
CArmModel::getForwardKinematics(const cv::Mat& q, cv::Mat& pose) const
{
    mImpl->getPoseFromJointAngles(q, pose);
}


/**
 * @brief CArmModel::getForwardKinematics
 * @param pose
 */
void
CArmModel::getForwardKinematics(cv::Mat& pose) const
{
    mImpl->getPoseFromJointAngles(mImpl->mCurrentJointAngles, pose);
}


/**
 * @brief CArmModel::getInverseKinematics
 * @param pose
 * @param q
 */
void
CArmModel::getInverseKinematics(const cv::Mat& pose, cv::Mat& q) const
{
    mImpl->getJointAnglesFromPose(pose, q);
}


/**
 * @brief CArmModel::getRobotJacobian
 * @param q
 * @param jacobian
 */
void
CArmModel::getRobotJacobian(const cv::Mat& q, cv::Mat& jacobian) const
{
    mImpl->getRobotJacobian(q, jacobian);
}


/**
 * @brief CArmModel::getRobotJacobian
 * @param q
 * @param jacobian
 */
void
CArmModel::getRobotJacobian(cv::Mat& jacobian) const
{
    mImpl->getRobotJacobian(mImpl->mCurrentJointAngles, jacobian);
}


/**
 * @brief CArmModel::get_eJe
 * @param dofPositions
 * @param jacobian
 */
void
CArmModel::get_eJe(const cv::Mat& dofPositions, cv::Mat& jacobian) const
{
    mImpl->get_eJe(dofPositions, jacobian);
}


/**
 * @brief CArmModel::get_cJe
 * @param jacobian
 */
void
CArmModel::get_cJe(cv::Mat& jacobian) const
{
    mImpl->get_cJe(jacobian);
}


/**
 * @brief CArmModel::get_cJe
 * @param dofPositions
 * @param jacobian
 */
void
CArmModel::get_cJe(const cv::Mat& dofPositions, cv::Mat& jacobian) const
{
    mImpl->get_cJe(dofPositions, jacobian);
}


/**
 * @brief CArmModel::get_fMe
 * @param fMe
 */
void
CArmModel::get_fMe(cv::Mat& fMe) const
{
    mImpl->get_fMe(fMe);
}


/**
 * @brief CArmModel::getJointVelocityFromCameraVelocity
 * @param cVc
 * @param q_dot
 */
void
CArmModel::getJointVelocityFromCameraVelocity(const cv::Mat& cVc, cv::Mat& q_dot) const
{
    mImpl->getJointVelocityFromCameraVelocity(cVc, q_dot);
}

} // namespace vs
