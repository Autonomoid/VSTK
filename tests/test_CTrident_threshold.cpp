/*******************************************
  Title: Image moments from colour-threshoold
  Description: IBVS control of ROV
*******************************************/

#define DEBUG

#ifdef DEBUG
    #define dbg(msg) std::cout << "[DBG] " << __TIME__ << ":" << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": " << msg << std::endl;
#else
    #define dbg(msg)
#endif

#include <algorithm>
#include <fstream>
#include <deque>
#include <vector>

#include "boost/program_options.hpp"
#include "boost/interprocess/managed_shared_memory.hpp"

#include "../../VS/include/utils.hpp"
#include "../../VS/include/CKeyPointMatcher.hpp"
#include "../../VS/include/CROSFrameGrabber.hpp"
#include "../../VS/include/CImageDecorator.hpp"
#include "../../VS/include/CVehicleInterface.hpp"
#include "../../VS/include/CArm.hpp"
#include "../../VS/include/CArmModel.hpp"
#include "../../VS/include/CLMoments.h"
#include "../../VS/include/mouse.hpp"
#include "../../VS/include/CAdaptiveGain.hpp"

namespace po = boost::program_options;
void setParameters();

int size = 20;
cv::Rect roi;
cv::Rect c_roi;
cv::Rect d_roi;
cv::Mat thresh;
cv::Point p1;
cv::Point p2;

// -- BEGIN PARAMETERS -- //

    // Threshodling params
    int r_min = 0;
    int r_max = 255;
    int g_min = 0;
    int g_max = 255;
    int b_min = 0;
    int b_max = 255;

    // ROV params
    double ROV_VEL_MAX[6];

    // Additional velocity screw damping factors
    double screw_factor[6];

// -- END PARAMETERS -- //

//void getMomentFeatures(cv::Mat& inputImage, cv::Mat& outputFeatures, cv::Mat& outputL);

int main(int argc, char *argv[])
{
  // Create a ROS node
  ros::init(argc, argv, "test_CRobustMatcher");
  ros::NodeHandle nh;


  // Setup options.
  setParameters();

  // Get current frame fromn shared memory
  boost::interprocess::managed_shared_memory msm(boost::interprocess::open_only, vs::utils::MEMORY_NAME);
  const vs::utils::SharedImageHeader* shared_image_header = msm.find<vs::utils::SharedImageHeader>("MatHeader").first;
  const cv::Mat currentFrame(shared_image_header->size, shared_image_header->type,
                             msm.get_address_from_handle(shared_image_header->handle));

  // Create a decorator
  CImageDecorator dec;
  std::stringstream text;

  // Create an output file
  std::ofstream outputFile("data.csv");
  outputFile << "e_v[0],e_v[1],e_vi[2],e_w[0],e_w[1],e_w[2],v[0],v[1],v[2],v[3],v[4],v[5],v_s[0],v_s[1],v_s[2],v_s[3],v_s[4],v_s[5],c_dist,d_dist,e_dist,e_distance_max,e_distance_ratio,c_orient,d_orient,e_orient,e_orient_max,e_orient_ratio" << std::endl;

  // Create an arm
  vs::CArm myArm(nh);
  myArm.setNDOF(7);

  // Create a vehicle
  vs::CVehicleInterface trident(nh);
  trident.setNDOF(9);

  // Create containers
  cv::Mat c_img;

  cv::Scalar mcolor(0,255,0);
  cv::Scalar kpcolor(0,0,255);

  // Set vehicle home pose
  cv::Mat home_pose(6, 1, CV_64F);
  home_pose.at<double>(0) = 2.5;
  home_pose.at<double>(1) = 2.0;
  home_pose.at<double>(2) = 1.9;
  home_pose.at<double>(3) = 0.0;
  home_pose.at<double>(4) = 0.0;
  home_pose.at<double>(0) = -0.0466;
  trident.setHomePose(home_pose);

  // Create track bars for setting RGB range
  cvNamedWindow("thresholding", CV_WINDOW_AUTOSIZE);
  cvCreateTrackbar("r_min","thresholding",&r_min,255);
  cvCreateTrackbar("r_max","thresholding",&r_max,255);
  cvCreateTrackbar("g_min","thresholding",&g_min,255);
  cvCreateTrackbar("g_max","thresholding",&g_max,255);
  cvCreateTrackbar("b_min","thresholding",&b_min,255);
  cvCreateTrackbar("b_max","thresholding",&b_max,255);

  int x_factor = 10;
  int y_factor = 10;
  int z_factor = 10;
  int roll_factor = 0;
  int pitch_factor = 0;
  int yaw_factor = 10;
  cvCreateTrackbar("x_factor","thresholding",&x_factor,10000);
  cvCreateTrackbar("y_factor","thresholding",&y_factor,10000);
  cvCreateTrackbar("z_factor","thresholding",&z_factor,10000);
  cvCreateTrackbar("roll_factor","thresholding",&roll_factor,10000);
  cvCreateTrackbar("pitch_factor","thresholding",&pitch_factor,10000);
  cvCreateTrackbar("yaw_factor","thresholding",&yaw_factor,10000);

  int iter = 0;

  vs::CAdaptiveGain lambda_x(0.0, 0.01, 0.0001);
  vs::CAdaptiveGain lambda_y(0.0, 0.01, 0.0001);
  vs::CAdaptiveGain lambda_z(0.0, 0.01, 0.0000001);

  vs::CAdaptiveGain lambda_roll(0.0, 0.01, 0.00001);
  vs::CAdaptiveGain lambda_pitch(0.0, 0.01, 0.00001);
  vs::CAdaptiveGain lambda_yaw(0.0, 0.01, 0.00001);

  // Specify desired features:
  // 1. Centred in the image (x = y = 0)
  // 2. Certain depth
  // 3. Roll, pitch & yaw = 0
  cv::Mat desiredFeatures = cv::Mat::zeros(6, 1, CV_64F);
  desiredFeatures.at<double>(0) = 0.0; // Initially translate to (0,0) to do Z rotation.
  desiredFeatures.at<double>(1) = 0.0;
  desiredFeatures.at<double>(2) = std::atof(argv[3]); //20000; // area
  desiredFeatures.at<double>(5) = std::atof(argv[4]); //0.0; // yaw angle

  double currentWaypoint = 0;

  // Loop forever
  while(true)
  {
    // Process ROS callback queue
    ros::spinOnce();

    // Get current image
    c_img = currentFrame.clone();

    // If the image isn't empty...
    if(c_img.empty()==false)
    {

      // Threshold image --> thresh is of type CV_8U
      // So each px contributes 1 to m00.
      cv::inRange(c_img, cv::Scalar(b_min, g_min, r_min), cv::Scalar(b_max, g_max, r_max), thresh);

      // Find contours from thresholded image
      std::vector<std::vector<cv::Point> > contours;
      std::vector<cv::Vec4i> hierarchy;
      cv::findContours(thresh, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

      // Find the convex hull object for each contour
      std::vector<cv::Point> combined_contours;

      std::vector<std::vector<cv::Point> > hull(contours.size());
      for(int i=0; i<contours.size(); ++i)
      {
          for(int j=0; j<contours[i].size(); ++j)
          {
              combined_contours.push_back(contours.at(i).at(j));
          }
      }

      // Get convex hull of the combined contours
      if(combined_contours.size() == 0)
      {
          std::cout << "No contours found" << std::endl;
      }
      cv::convexHull(cv::Mat(combined_contours), hull[0], false);


      // Draw the individual contours and the hull
      cv::RNG rng;
      cv::Mat drawing = cv::Mat::zeros(thresh.size(), CV_8UC3);
      for(int i=0; i<contours.size(); ++i)
      {
        cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255));
        cv::drawContours(drawing, contours, i, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
      }
      cv::drawContours(drawing, hull, 0, cv::Scalar(255,255,255), 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());

      // --> hull[0] is the bounding hull over all contours.

      // Show in a window
      cv::namedWindow( "Convex Hulls", CV_WINDOW_AUTOSIZE );
      cv::imshow( "Convex Hulls", drawing);

      //cv::namedWindow( "Thresholded Image", CV_WINDOW_AUTOSIZE );
      //cv::imshow( "Thresholded Image", 255*thresh);

      // Calculate the moment features
      cv::Mat currentFeatures;
      vs::utils::getMomentFeatures(hull[0], currentFeatures);

      // Move (0,0) to image centre
      currentFeatures.at<double>(0) = currentFeatures.at<double>(0) - c_img.cols / 2.0;
      currentFeatures.at<double>(1) = currentFeatures.at<double>(1) - c_img.rows / 2.0;

      currentFeatures.at<double>(3) = 0.0;
      currentFeatures.at<double>(4) = 0.0;

      // Compute the feature error
      cv::Mat error = currentFeatures - desiredFeatures;

      // See Mahony, Corke, Chaumette (2002) for  computing the z-axis error.
      //error.at<double>(2) = std::log(sqrt(currentFeatures.at<double>(2)) - sqrt(desiredFeatures.at<double>(2)));
      error.at<double>(2) = (sqrt(currentFeatures.at<double>(2)) - sqrt(desiredFeatures.at<double>(2)));

      // Compute gains
      cv::Mat gain = cv::Mat::zeros(6, 1, error.type());
      gain.at<double>(0) = lambda_x(std::abs(error.at<double>(0)));
      gain.at<double>(1) = lambda_y(std::abs(error.at<double>(1)));
      gain.at<double>(2) = lambda_z(std::abs(error.at<double>(2)));

      gain.at<double>(3) = lambda_roll(std::abs(error.at<double>(3)));
      gain.at<double>(4) = lambda_pitch(std::abs(error.at<double>(4)));
      gain.at<double>(5) = lambda_yaw(std::abs(error.at<double>(5)));

      // Find the plane of the target at depth Z
      // Ax + By + C = 1/Z

      cv::Mat H(4, 4, CV_64F);
      myArm.mModel->getForwardKinematics(H);
      double arm_z = H.at<double>(2,3);

      cv::Mat currentPose;
      trident.getDOFPositions(currentPose);

      double A = 0.0;
      double B = 0.0;
      double C = 1.0/(4.57 - arm_z - currentPose.at<double>(2)); // Z = 1/C m
      std::cout << "1/C = " << 1/C << std::endl;

      // Compute L_inv
      CLMoments lmoments;
      int order = 2;
      lmoments.setPlane(A, B, C);
      lmoments.setImage<std::vector<cv::Point> >(hull[0], order);
      cv::Mat L = cv::Mat(6, 6, CV_64F);
      L = lmoments.getInteractionMatrix();
      //cv::Mat L_inv = L.inv(cv::DECOMP_SVD);
      cv::Mat L_inv = L.t();

      std::cout << "L = " << std::endl << L << std::endl;

      // Compute screw
      cv::Mat screw = (L_inv * error).mul(gain);

      //Map the velocity screw from camera frame to base frame." << std::endl;
      cv::Mat bWc(6, 6, CV_64F);
      vs::utils::HTransformToTwist(H, bWc);
      cv::Mat screw2 = /*bWc **/ screw;

      // Map from cv::mat to std::vector and print
      std::cout << "[+] Checking velocity screw for NaN entries." << std::endl;
      for(int i = 0; i < 6; i++)
      {
          // Check for NaNs
          if(screw2.at<double>(i) != screw2.at<double>(i))
          {
              screw2.at<double>(i) = 0;
              //exit(1);
          }
      }

      // Rescale the gain via GUI sliders
      screw2.at<double>(0) = screw2.at<double>(0) * (double)x_factor;
      screw2.at<double>(1) = screw2.at<double>(1) * (double)y_factor;

      // Only zoom and yaw after centring at (0,0)
      double x_error_thresh = 10.0;
      double y_error_thresh = 10.0;

      if((std::abs(error.at<double>(0)) < x_error_thresh) && (std::abs(error.at<double>(1)) < y_error_thresh))
      {
          screw2.at<double>(2) = screw2.at<double>(2) * (double)z_factor;

          screw2.at<double>(3) = screw2.at<double>(3) * (double)roll_factor;
          screw2.at<double>(4) = screw2.at<double>(4) * (double)pitch_factor;
          screw2.at<double>(5) = screw2.at<double>(5) * (double)yaw_factor;
      }
      else
      {
          screw2.at<double>(2) = 0.0;
          screw2.at<double>(3) = 0.0;
          screw2.at<double>(4) = 0.0;
          screw2.at<double>(5) = 0.0;
      }

      // Only translate after zoom and yaw finished.
      double z_error_thresh = 10.0;
      double yaw_error_thresh = 0.1;

      if((std::abs(error.at<double>(2)) < z_error_thresh) && (std::abs(error.at<double>(5)) < yaw_error_thresh))
      {
          desiredFeatures.at<double>(0) = std::atof(argv[1]);
          desiredFeatures.at<double>(1) = std::atof(argv[2]);
      }

      // Saturate velocity screw
      // Check each velocity screw component.
      for(uint i = 0; i < 6; ++i)
      {
        double v_i = fabs(screw2.at<double>(i));
        //double v_max_i = fabs(ROV_VEL_MAX[i]);
        double v_max_i = fabs(0.5);

        if(v_i > v_max_i)
        {
          double scale_i = double(v_max_i) / double(v_i);
          screw2.at<double>(i) = screw2.at<double>(i) * scale_i;
        } // if()
      } // for()

      // Send screw to vehicle
      std::cout << "[+] Sending velocity screw request to vehicle." << std::endl;
//      trident.setDOFVelocities(screw2);

      // If converged, set next waypoint
      if((std::abs(error.at<double>(2)) < z_error_thresh) && (std::abs(error.at<double>(5)) < yaw_error_thresh)
          && (std::abs(error.at<double>(0)) < x_error_thresh) && (std::abs(error.at<double>(1)) < y_error_thresh))
      {
          ++currentWaypoint;

          if(currentWaypoint == 1)
          {
              // Go to red box
              r_min = 85;
              r_max = 255;
              g_min = 0;
              g_max = 102;
              b_min = 0;
              b_max = 55;
          }

          if(currentWaypoint == 2)
          {
              // Go to green box
              r_min = 0;
              r_max = 133;
              g_min = 130;
              g_max = 255;
              b_min = 0;
              b_max = 106;
          }

          if(currentWaypoint == 3)
          {
              // Go to blue box
              r_min = 0;
              r_max = 70;//76;
              g_min = 0;
              g_max = 60;//90;
              b_min = 110;
              b_max = 255;
          }

          if(currentWaypoint == 4)
          {
              // Go to green box
              r_min = 0;
              r_max = 133;
              g_min = 130;
              g_max = 255;
              b_min = 0;
              b_max = 106;
          }

          if(currentWaypoint == 5)
          {
              // Go to red box
              r_min = 85;
              r_max = 255;
              g_min = 0;
              g_max = 102;
              b_min = 0;
              b_max = 55;
          }

          if(currentWaypoint == 6)
          {
              // Go to blue box
              r_min = 0;
              r_max = 70;//76;
              g_min = 0;
              g_max = 60;//90;
              b_min = 110;
              b_max = 255;

              // Reset
              currentWaypoint = 0;
          }

      }

      // Display results
      dec.display();
      cv::waitKey(30);

      /////////////////////////// Diagnostic info //////////////////////////////

      std::cout << "\nCurrent features:" << std::endl;
      std::cout << "x_g = " << currentFeatures.at<double>(0) << std::endl;
      std::cout << "y_g = " << currentFeatures.at<double>(1) << std::endl;
      std::cout << "a = " << currentFeatures.at<double>(2) << std::endl;
      std::cout << "normalized a = " << currentFeatures.at<double>(2)/(c_img.rows*c_img.cols) << std::endl;
      std::cout << "P_2 = " << currentFeatures.at<double>(3) << std::endl;
      std::cout << "P_3 = " << currentFeatures.at<double>(4) << std::endl;
      std::cout << "theta = " << currentFeatures.at<double>(5) << " rad, "
                << currentFeatures.at<double>(5) * (180.0/3.14159265) << " deg" << std::endl << std::endl;

      std::cout << "Desired features:" << std::endl;
      std::cout << "x_g* = " << desiredFeatures.at<double>(0) << std::endl;
      std::cout << "y_g* = " << desiredFeatures.at<double>(1) << std::endl;
      std::cout << "a* = " << desiredFeatures.at<double>(2) << std::endl;
      std::cout << "P_2* = " << desiredFeatures.at<double>(3) << std::endl;
      std::cout << "P_3* = " << desiredFeatures.at<double>(4) << std::endl;
      std::cout << "theta* = " << desiredFeatures.at<double>(5) << " rad, "
                << desiredFeatures.at<double>(5) * (180.0/3.14159265) << " deg" << std::endl << std::endl;

      std::cout << "[+] Error = " << error << std::endl;
      std::cout << "[+] Adaptive gains = " << gain << std::endl;
      std::cout << "[+] Depth (Z) = " << C << std::endl;
      std::cout << "[+] L pseudo-inverse = " << std::endl << L_inv << std::endl;
      std::cout << "[+] Screw (raw) = " << screw << std::endl;

      // Print final screw values
      for(int i = 0; i < 6; i++)
      {
          std::cout << "Final screw[" << i << "] = " << screw2.at<double>(i) << std::endl;
      }


    } // if images is not empty
    else
    {
        std::cout << "[!] No image detected" << std::endl;
    }

    ++iter;

  } // while(true)

  outputFile.close();

  return 0;
}


/**
 * @brief
 */
void
setParameters()
{
  po::options_description desc("Options");
  desc.add_options()

    // Thresholding params
    ("thresholding.r_min", po::value< int >( &r_min )->required(), "r_min")
    ("thresholding.r_max", po::value< int >( &r_max )->required(), "r_max")
    ("thresholding.g_min", po::value< int >( &g_min )->required(), "g_min")
    ("thresholding.g_max", po::value< int >( &g_max )->required(), "g_max")
    ("thresholding.b_min", po::value< int >( &b_min )->required(), "b_min")
    ("thresholding.b_max", po::value< int >( &b_max )->required(), "b_max")

    // Vehicle params
    ("vehicle.vel_max_x", po::value< double >( &ROV_VEL_MAX[0] )->required(), "ROV vel max x")
    ("vehicle.vel_max_y", po::value< double >( &ROV_VEL_MAX[1] )->required(), "ROV vel max y")
    ("vehicle.vel_max_z", po::value< double >( &ROV_VEL_MAX[2] )->required(), "ROV vel max z")
    ("vehicle.vel_max_roll", po::value< double >( &ROV_VEL_MAX[3] )->required(), "ROV vel max roll")
    ("vehicle.vel_max_pitch", po::value< double >( &ROV_VEL_MAX[4] )->required(), "ROV vel max pitch")
    ("vehicle.vel_max_yaw", po::value< double >( &ROV_VEL_MAX[5] )->required(), "ROV vel max yaw")

    // Visual servoing params
    ("vs.screw_factor_x", po::value< double >( &screw_factor[0] )->required(), "screw_factor[0]")
    ("vs.screw_factor_y", po::value< double >( &screw_factor[1] )->required(), "screw_factor[1]")
    ("vs.screw_factor_z", po::value< double >( &screw_factor[2] )->required(), "screw_factor[2]")
    ("vs.screw_factor_roll", po::value< double >( &screw_factor[3] )->required(), "screw_factor[3]")
    ("vs.screw_factor_pitch", po::value< double >( &screw_factor[4] )->required(), "screw_factor[4]")
    ("vs.screw_factor_yaw", po::value< double >( &screw_factor[5] )->required(), "screw_factor[5]")
  ;

  // Load setting file.
  po::variables_map vm;
  std::ifstream settings_file( "../settings/trident_settings_thresh.ini" , std::ifstream::in );
  po::store( po::parse_config_file( settings_file , desc ), vm );
  settings_file.close();
  po::notify( vm );

} // setParameters()

