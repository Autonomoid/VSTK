/////////////////////////////////////////////////////////////////
//
// Name: test_illum

//#define DEBUG

#ifdef DEBUG
    #define dbg(msg) std::cout << "[DBG] " << __TIME__ << ":" << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": " << msg << std::endl;
#else
    #define dbg(msg)
#endif

#include <math.h>

#include <boost/interprocess/managed_shared_memory.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../../VS/include/utils.hpp"
#include "../../VS/include/CVehicleInterface.hpp"

int main(int argc, char* argv[])
{
  // Create a ROS node
  ros::init(argc, argv, "test_illum");
  ros::NodeHandle nh;

  // Open Shared Memory Manager
  boost::interprocess::managed_shared_memory msm(boost::interprocess::open_only, vs::utils::MEMORY_NAME);

  // Open Shared Mat Header
  const vs::utils::SharedImageHeader* shared_image_header = msm.find<vs::utils::SharedImageHeader>("MatHeader").first;

  // Make shared and read only (const) cv::Mat
  // Header information (size and type) is also provided from Shared Memory
  const cv::Mat currentFrame(shared_image_header->size, shared_image_header->type,
                             msm.get_address_from_handle(shared_image_header->handle));

  cv::Mat output;

  // Beam parameters
  int mode = 0;
  int beamBrightness_coeff = 1.0;
  int beamBrightness_exponent = 1.0;
  int beamWidth = 200.0;
  int beamHeight = 200.0;
  int beamCenterX = (currentFrame.cols+1)/2.0;
  int beamCenterY = (currentFrame.rows+1)/2.0;

  cvNamedWindow("IlluminationX", CV_WINDOW_AUTOSIZE);
  cvCreateTrackbar("beamProfile", "IlluminationX",&mode,2);
  cvCreateTrackbar("beamBrightness_coeff", "IlluminationX",&beamBrightness_coeff,10);
  cvCreateTrackbar("beamBrightness_exponent", "IlluminationX",&beamBrightness_exponent,20);
  cvCreateTrackbar("beamWidth", "IlluminationX",&beamWidth,500);
  cvCreateTrackbar("beamHeight", "IlluminationX",&beamHeight,500);
  cvCreateTrackbar("beamCenterX", "IlluminationX",&beamCenterX,currentFrame.cols-1);
  cvCreateTrackbar("beamCenterY", "IlluminationX",&beamCenterY,currentFrame.rows-1);

  // Scene parameters
  double theta_x = 0;
  double theta_y = 0;
  double targetDistance = 0.0;
  double ztarget = 2.0;
  double z = 0;

  // Create a vehicle
  vs::CVehicleInterface trident(nh);
  trident.setNDOF(6);
  cv::Mat pose;

  // final_ID of shared matrix
  int frameID = 0;

  // Loop until user presses 'q'.
  while ('q' != cv::waitKey(1))
  {
    ros::spinOnce();
    output = currentFrame.clone();

    // If a new frame is available
    if(frameID < shared_image_header->frameID)
    {
      frameID = shared_image_header->frameID;

      // Get pitch and roll.
      trident.getDOFPositions(pose);
      theta_x = pose.at<double>(3);
      theta_y = pose.at<double>(4);
      z = pose.at<double>(2);
      targetDistance = z-ztarget;

      ///////////////////// ILLUMINATION SIM /////////////////////
      vs::utils::illuminate(currentFrame, output, mode, theta_x, theta_y, targetDistance,
                 beamBrightness_coeff, beamBrightness_exponent, beamCenterX, beamCenterY,
                 beamHeight, beamWidth);

      // Display results
      cv::imshow("Input", currentFrame);
      cv::imshow("Output", output);
    }
  }

  return 0;
}


