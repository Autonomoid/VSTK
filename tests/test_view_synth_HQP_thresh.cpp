
/*******************************************
  Title: Image moments from colour-threshoold
  Description: IBVS control of ROV
*******************************************/

//#define DEBUG

#ifdef DEBUG
    #define dbg(msg) std::cout << "[DBG] " << __TIME__ << ":" << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": " << msg << std::endl;
#else
    #define dbg(msg)
#endif

#include <algorithm>
#include <fstream>
#include <deque>
#include <vector>
#include <iostream>

#include "boost/program_options.hpp"
#include "boost/interprocess/managed_shared_memory.hpp"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

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
#include "../../VS/include/CHQPSolver.hpp"

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
    int r_min = 85;
    int r_max = 255;
    int g_min = 0;
    int g_max = 100;
    int b_min = 0;
    int b_max = 55;


    // ROV params
    double ROV_VEL_MAX[6];

    // Additional velocity screw damping factors
    double screw_factor[6];

// -- END PARAMETERS -- //

//void getMomentFeatures(cv::Mat& inputImage, cv::Mat& outputFeatures, cv::Mat& outputL);

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "test_view_synth_HQP_thresh");
  ros::NodeHandle nh;

  setParameters();


/////

  // Define camera pose parameters
  int w = 20;
  int l = 60;
  int h = 15;

  int rx = 180;
  int ry = 180;
  int rz = 180;

  int tx = 500;
  int ty = 500;
  int tz = 500;

  int fx = 400;
  int fy = 300;
  int cx = 400;
  int cy = 300;

  // Create trackbars/sliders for adjusting the pose parameters.
  cvNamedWindow("Transformation", CV_WINDOW_AUTOSIZE);

  cvCreateTrackbar("box width /cm","Transformation",&w,100);
  cvCreateTrackbar("box length /cm","Transformation",&l,100);
  cvCreateTrackbar("box height /cm","Transformation",&h,100);

  cvCreateTrackbar("fx","Transformation",&fx,800);
  cvCreateTrackbar("fy","Transformation",&fy,600);
  cvCreateTrackbar("cx","Transformation",&cx,800);
  cvCreateTrackbar("cy","Transformation",&cy,600);

  cvCreateTrackbar("tx","Transformation",&tx,1000);
  cvCreateTrackbar("ty","Transformation",&ty,1000);
  cvCreateTrackbar("tz","Transformation",&tz,1000);
  cvCreateTrackbar("rx","Transformation",&rx,360);
  cvCreateTrackbar("ry","Transformation",&ry,360);
  cvCreateTrackbar("rz","Transformation",&rz,360);

  // Define the world coordinates of the target object (box).
  std::vector<cv::Point3_<double> > targetPoints;

  double width = double(w)/100.0; // x
  double length = double(l)/100.0; // y
  double height = double(h)/100.0; // z

  targetPoints.push_back(cv::Point3_<double>(width/2,-length/2,-height));
  targetPoints.push_back(cv::Point3_<double>(width/2,length/2,-height));
  targetPoints.push_back(cv::Point3_<double>(-width/2,length/2,-height));
  targetPoints.push_back(cv::Point3_<double>(-width/2,-length/2,-height));

  targetPoints.push_back(cv::Point3_<double>(width/2,-length/2,-2*height));
  targetPoints.push_back(cv::Point3_<double>(width/2,length/2,-2*height));
  targetPoints.push_back(cv::Point3_<double>(-width/2,length/2,-2*height));
  targetPoints.push_back(cv::Point3_<double>(-width/2,-length/2,-2*height));

  // GetCurrentFrameFromSharedMemory(cv::Mat& currentFrame)
  boost::interprocess::managed_shared_memory msm(boost::interprocess::open_only, vs::utils::MEMORY_NAME);
  const vs::utils::SharedImageHeader* shared_image_header = msm.find<vs::utils::SharedImageHeader>("MatHeader").first;
  cv::Mat currentFrame(shared_image_header->size, shared_image_header->type,
                             msm.get_address_from_handle(shared_image_header->handle));

  CImageDecorator dec;

  std::ofstream outputFile("data.csv");
  outputFile << "e_v[0],e_v[1],e_vi[2],e_w[0],e_w[1],e_w[2],v[0],v[1],v[2],v[3],v[4],v[5],v_s[0],v_s[1],v_s[2],v_s[3],v_s[4],v_s[5],c_dist,d_dist,e_dist,e_distance_max,e_distance_ratio,c_orient,d_orient,e_orient,e_orient_max,e_orient_ratio" << std::endl;

  // Create an arm
  vs::CArm arm(nh);
  arm.setNDOF(7);
  cv::Mat startDOFPositions;
  arm.mInterface->getDOFPositions(startDOFPositions);

  // Create a vehicle
  vs::CVehicleInterface trident(nh);
  trident.setNDOF(6);

  // Create containers
  cv::Mat c_img;

  // Give a constant gain of 0.1 on all DOF.
  vs::CAdaptiveGain lambda_x(0.1, 1.0, 0.0);
  vs::CAdaptiveGain lambda_y(0.1, 1.0, 0.0);
  vs::CAdaptiveGain lambda_z(0.1, 1.0, 0.0);

  vs::CAdaptiveGain lambda_roll(0.1, 1.0, 0.0);
  vs::CAdaptiveGain lambda_pitch(0.1, 1.0, 0.0);
  vs::CAdaptiveGain lambda_yaw(0.1, 1.0, 0.0);

  // Specify desired features:
  // 1. Centred in the image (x = y = 0)
  // 2. Certain depth
  // 3. Roll, pitch & yaw = 0
  cv::Mat desiredFeatures = cv::Mat::zeros(6, 1, CV_64F);


  // Wait for pose to initialize.
  while(startDOFPositions.empty())
  {
      ros::spinOnce();
      arm.mInterface->getDOFPositions(startDOFPositions);
  }

  // Loop forever
  int iter = 0;
  while(true)
  {
      std::cout << "\n\nIteration = " << iter << std::endl;

      // Convert Camera rotation angles from deg to rad
      double theta_x = double(rx-180)*M_PI/180.0;
      double theta_y = double(ry-180)*M_PI/180.0;
      double theta_z = double(rz-180)*M_PI/180.0;

      // Define the camera rotation matrix
      cv::Mat rotation_x = cv::Mat::zeros(3, 3, cv::DataType<double>::type);
      rotation_x.at<double>(0,0) = 1;
      rotation_x.at<double>(1,1) = std::cos(theta_x);
      rotation_x.at<double>(1,2) = -std::sin(theta_x);
      rotation_x.at<double>(2,1) = std::sin(theta_x);
      rotation_x.at<double>(2,2) = std::cos(theta_x);

      cv::Mat rotation_y = cv::Mat::zeros(3, 3, cv::DataType<double>::type);
      rotation_y.at<double>(0,0) = std::cos(theta_y);
      rotation_y.at<double>(0,2) = std::sin(theta_y);
      rotation_y.at<double>(1,1) = 1;
      rotation_y.at<double>(2,0) = -std::sin(theta_y);
      rotation_y.at<double>(2,2) = std::cos(theta_y);

      cv::Mat rotation_z = cv::Mat::zeros(3, 3, cv::DataType<double>::type);
      rotation_z.at<double>(0,0) = std::cos(theta_z);
      rotation_z.at<double>(0,1) = -std::sin(theta_z);
      rotation_z.at<double>(1,0) = std::sin(theta_z);
      rotation_z.at<double>(1,1) = std::cos(theta_z);
      rotation_z.at<double>(2,2) = 1;

      cv::Mat rotation = rotation_x * rotation_y * rotation_z;

      // Convert the rotation matrix to the axis-angle rotation vector
      cv::Mat rotationVector;
      cv::Rodrigues(rotation, rotationVector);

      // Define the camera translation
      cv::Mat translationVector = cv::Mat::zeros(3, 1, cv::DataType<double>::type);
      translationVector.at<double>(0, 0) = double(tx-500)/10.0; // 10cm increments
      translationVector.at<double>(1, 0) = double(ty-500)/10.0;
      translationVector.at<double>(2, 0) = double(tz-500)/10.0;

      // Define the camera matrix
      cv::Mat cameraMatrix = cv::Mat::eye(3, 3, cv::DataType<double>::type);
      int imageWidth = 800;
      int imageHeight = 600;
      cameraMatrix.at<double>(0,0) = (double)fx;
      cameraMatrix.at<double>(1,1) = (double)fy;
      cameraMatrix.at<double>(0,2) = (double)cx;
      cameraMatrix.at<double>(1,2) = (double)cy;

      // Set distortion coefficients
      cv::Mat distortion = cv::Mat::zeros(4, 1, cv::DataType<double>::type);

      // Project 3D points onto the camera plane.
      std::vector<cv::Point_<double> > imagePoints;
      cv::projectPoints(targetPoints, rotationVector, translationVector, cameraMatrix, distortion, imagePoints);

      // Plot projected points
      cv::Mat image = cv::Mat::zeros(imageHeight, imageWidth, CV_8U);
      for(std::vector<cv::Point_<double> >::iterator it = imagePoints.begin(); it != imagePoints.end(); ++it)
      {
          cv::circle(image, (*it), 5, cv::Scalar(255,255,255), 1, 8);
      }

      // Plot lines connecting the points
      for(int i=0; i<4; ++i)
      {
          cv::line(image, imagePoints.at(i), imagePoints.at((i+1)%4), cv::Scalar(255, 255, 255), 2, 8);
          cv::line(image, imagePoints.at(i+4), imagePoints.at((i+5)<8?(i+5):4), cv::Scalar(255, 255, 255), 2, 8);
          cv::line(image, imagePoints.at(i), imagePoints.at(i+4), cv::Scalar(255, 255, 255), 2, 8);
      }

      // Define the polygons representing the faces of the target.
      cv::Point pt[1][4];

      for(int i=0; i<4; ++i)
      {
          // Front face
          pt[0][i].x = imagePoints.at(i).x;
          pt[0][i].y = imagePoints.at(i).y;
      }

      // Group the polygons
      const cv::Point* ppt[1] = {pt[0]};

      // Number of points defining each polygon/face.
      int npt[] = {4};

      // Do the fill.
      cv::Mat image2 = cv::Mat::zeros(imageHeight, imageWidth, CV_8U);
      cv::fillPoly(image2, ppt, npt, 1, cv::Scalar(255,255,255), 8);

      // Compute moments
      cv::Moments mo = cv::moments(image2);
      double area = mo.m00;
      double centroid_x = mo.m10 / area;
      double centroid_y = mo.m01 / area;

      double mu_11 = mo.mu11 / area;
      double mu_20 = mo.mu20 / area;
      double mu_02 = mo.mu02 / area;
      double theta = 0.5 * cv::fastAtan2(2.0 * mu_11, mu_20 - mu_02);

      std::cout << "\narea = " << area << std::endl;
      std::cout << "normalized area = " << area/(255*imageWidth*imageHeight) << std::endl;
      std::cout << "centroid x = " << centroid_x << std::endl;
      std::cout << "centroid y = " << centroid_y << std::endl;
      std::cout << "theta = " << vs::utils::radToDeg(theta) << std::endl;

      // Darw on the centroid
      cv::circle(image, cv::Point_<double>(centroid_x, centroid_y), 5, cv::Scalar(255,255,255), 1, 8);

      // Display the results
      cv::imshow("projectedPoints", image);
      cv::imshow("faces", image2);

      desiredFeatures.at<double>(0) = 0.0; // x co-ordiante /px
      desiredFeatures.at<double>(1) = 0.0; // y co-ordiante /px
      desiredFeatures.at<double>(2) = 2000.0; // area /px
      desiredFeatures.at<double>(5) = theta; // yaw angle


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

// End of image pre-processing //

          // Calculate the moment features
          cv::Mat currentFeatures;
          vs::utils::getMomentFeatures(hull[0], currentFeatures);

/*
          cv::Mat test = cv::Mat(currentFrame.rows, currentFrame.cols, CV_8UC3);
          cv::drawContours(test, hull, 0, cv::Scalar(255,255,255), CV_FILLED);
          cv::Mat test2;
          cv::cvtColor(test,test2, CV_8UC1);
          vs::utils::getMomentFeatures2(test2, currentFeatures);
*/

          // Move the point (0,0) to the image centre
          currentFeatures.at<double>(0) = currentFeatures.at<double>(0) - c_img.cols / 2.0;
          currentFeatures.at<double>(1) = currentFeatures.at<double>(1) - c_img.rows / 2.0;

          // Set current roll and pitch to 0.0 as they are not being controlled.
          currentFeatures.at<double>(3) = 0.0;
          currentFeatures.at<double>(4) = 0.0;

          std::cout << "Current features " << std::endl << currentFeatures << std::endl;

          // Compute the current feature error
          cv::Mat error = currentFeatures - desiredFeatures;

          // See Mahony, Corke & Chaumette (2002) for computing z-axis error.
          error.at<double>(2) = (sqrt(currentFeatures.at<double>(2)) - sqrt(desiredFeatures.at<double>(2)));

          // Get the pose of the camera in the world frame (to get pitch and roll)
          cv::Mat wMf, fMc, wMc, cameraPose, vehiclePose;
          trident.getDOFPositions(vehiclePose);
          vs::utils::PoseToHTransform(vehiclePose, wMf);
          arm.mModel->getForwardKinematics(fMc);
          vs::utils::combineHTransforms(wMf, fMc, wMc);
          vs::utils::HTransformToPose(wMc, cameraPose);
          error.at<double>(3) = cameraPose.at<double>(3);
          error.at<double>(4) = cameraPose.at<double>(4);
          std::cout << "cameraPose = " << cameraPose << std::endl;

          std::cout << "[+] Error = " << error << std::endl;

/*
          // Only enable z and yaw DOFs after centering x and y at (0,0).
          // This is to ensure decoupling in L.
          double x_error_thresh = 5.0;
          double y_error_thresh = 5.0;
          if((std::abs(error.at<double>(0)) > x_error_thresh) || (std::abs(error.at<double>(1)) > y_error_thresh))
          {
              error.at<double>(2) = 0.0;
              error.at<double>(3) = 0.0;
              error.at<double>(4) = 0.0;
              error.at<double>(5) = 0.0;

              std::cout << "[+] Error --> " << error << std::endl;
          }
*/

          // Use the current error to update the adaptive gains.
          cv::Mat gain = cv::Mat::zeros(6, 1, error.type());
          gain.at<double>(0) = lambda_x(std::abs(error.at<double>(0)));
          gain.at<double>(1) = lambda_y(std::abs(error.at<double>(1)));
          gain.at<double>(2) = lambda_z(std::abs(error.at<double>(2)));

          gain.at<double>(3) = lambda_roll(std::abs(error.at<double>(3)));
          gain.at<double>(4) = lambda_pitch(std::abs(error.at<double>(4)));
          gain.at<double>(5) = lambda_yaw(std::abs(error.at<double>(5)));

          // Calculate the parameters of the plane of the target.
          // Ax + By + C = 1/Z
          cv::Mat fMe(4, 4, CV_64F);
          arm.mModel->get_fMe(fMe);
          double arm_z = fMe.at<double>(2,3);

          cv::Mat currentPose;
          trident.getDOFPositions(currentPose);

          // Z = 1/C m
          double A = 0.0;
          double B = 0.0;
          double C = 1.0/(4.57 - arm_z - currentPose.at<double>(2));
          std::cout << "1/C = " << 1/C << std::endl;

          // Compute L_inv
          CLMoments lmoments;
          int order = 2; // Using moments up to order 2.
          lmoments.setPlane(A, B, C);
          lmoments.setImage<std::vector<cv::Point> >(hull[0], order);
          cv::Mat L = cv::Mat(6, 6, CV_64F);
          L = lmoments.getInteractionMatrix();
          std::cout << "L = " << std::endl << L << std::endl;

          ////////////////////////////////////////////////////////////////////////////
          // Build HQP                                                              //
          ////////////////////////////////////////////////////////////////////////////

          cv::Mat J;
          arm.mModel->get_cJe(J);
          std::cout << "cJe = " << std::endl << J << std::endl;

          vs::CHQPSolver cascade;
          cascade.setEpsilon(0.000001);

                    cv::Mat dofPositions;
          arm.mInterface->getDOFPositions(dofPositions);
          double alpha = 1.0;

          ////////////////////////////////////////////////////////////////
          // Lower joint angle limits, q_i > lower_lim                  //
          ////////////////////////////////////////////////////////////////
          cv::Mat C1 = -1.0 * cv::Mat::eye(7, 7, CV_64F);
          cv::Mat d1 = cv::Mat::zeros(7, 1, CV_64F);

          std::vector<double> lower_lim;
          lower_lim.push_back(vs::utils::degToRad(-170));
          lower_lim.push_back(vs::utils::degToRad(-90));
          lower_lim.push_back(vs::utils::degToRad(-145));
          lower_lim.push_back(vs::utils::degToRad(-90));
          lower_lim.push_back(vs::utils::degToRad(-150));
          lower_lim.push_back(vs::utils::degToRad(-90));
          lower_lim.push_back(vs::utils::degToRad(-190));

          for(int i=0; i<7; ++i)
          {
              d1.at<double>(i) = alpha * (dofPositions.at<double>(i) - lower_lim.at(i));
          }

          ////////////////////////////////////////////////////////////////
          // Upper joint angle limits, q_i < upper_lim                  //
          ////////////////////////////////////////////////////////////////
          cv::Mat C2 = cv::Mat::eye(7, 7, CV_64F);
          cv::Mat d2 = cv::Mat::zeros(7, 1, CV_64F);

          std::vector<double> upper_lim;
          upper_lim.push_back(vs::utils::degToRad(170));
          upper_lim.push_back(vs::utils::degToRad(90));
          upper_lim.push_back(vs::utils::degToRad(145));
          upper_lim.push_back(vs::utils::degToRad(90));
          upper_lim.push_back(vs::utils::degToRad(150));
          upper_lim.push_back(vs::utils::degToRad(90));
          upper_lim.push_back(vs::utils::degToRad(190));

          for(int i=0; i<7; ++i)
          {
              d2.at<double>(i) = alpha * (upper_lim.at(i) - dofPositions.at<double>(i));
          }

          ////////////////////////////////////////////////////////////////
          // Lower joint speed limits, q_i_dot > lower_speed_lim        //
          ////////////////////////////////////////////////////////////////
          cv::Mat C5 = -1.0 * cv::Mat::eye(7, 7, CV_64F);
          cv::Mat d5 = cv::Mat::zeros(7, 1, CV_64F);

          double lower_speed_lim = -0.1; // rad/s
          for(int i=0; i<7; ++i)
          {
              d5.at<double>(i) = -lower_speed_lim;
          }

          ////////////////////////////////////////////////////////////////
          // Upper joint speed limits, q_i_dot > upper_speed_lim        //
          ////////////////////////////////////////////////////////////////
          cv::Mat C6 = cv::Mat::eye(7, 7, CV_64F);
          cv::Mat d6 = cv::Mat::zeros(7, 1, CV_64F);

          double upper_speed_lim = 0.1; // rad/s
          for(int i=0; i<7; ++i)
          {
              d6.at<double>(i) = upper_speed_lim;
          }

          ////////////////////////////////////////////////////////////////
          // Singularity Avoidance                                      //
          ////////////////////////////////////////////////////////////////
          cv::Mat C3 = cv::Mat::zeros(1, 7, CV_64F);
          cv::Mat d3 = cv::Mat::zeros(1, 1, CV_64F);

          double minJdet = 1.0; // 1e-4
          double s0 = cv::determinant(J*J.t());

          std::cout << "s0 = " << s0 << std::endl;

          d3.at<double>(0) = 0.2 * (s0 - minJdet);

          double dq = vs::utils::degToRad(0.1);

          double s1 = 0.0;
          double s2 = 0.0;

          for(int k=0; k<7; ++k)
          {
              arm.mInterface->getDOFPositions(dofPositions);

              dofPositions.at<double>(k) += dq;
              cv::Mat Jtmp1 = cv::Mat::zeros(7, 6, CV_64F);
              arm.mModel->get_eJe(dofPositions, Jtmp1);

              dofPositions.at<double>(k) -= 2*dq;
              cv::Mat Jtmp2 = cv::Mat::zeros(7, 6, CV_64F);
              arm.mModel->get_eJe(dofPositions, Jtmp2);

              s1 = cv::determinant(Jtmp1*Jtmp1.t());
              s2 = cv::determinant(Jtmp2*Jtmp2.t());

              C3.at<double>(k) = -(s1 - s2)/(2*dq);
          }

          ////////////////////////////////////////////////////////////////
          // Main servoing task (L*J*q = -lambda*e)                     //
          ////////////////////////////////////////////////////////////////

          cv::Mat A1 =  /* L * */ J;
          std::cout << "A1 = L * cJe = " << std::endl << A1 << std::endl;

          cv::Mat B1 = cv::Mat::zeros(6, 1, CV_64F);
          B1.at<double>(0,0) = 0.1 * -gain.at<double>(0) * error.at<double>(1);
          B1.at<double>(1,0) = 0.1 * gain.at<double>(1) * error.at<double>(0);
          B1.at<double>(2,0) = -gain.at<double>(2) * error.at<double>(2);

          B1.at<double>(3,0) = 0.0; //100.0 * -gain.at<double>(4) * error.at<double>(4);
          B1.at<double>(4,0) = 0.0; //-gain.at<double>(4) * error.at<double>(4);
          B1.at<double>(5,0) = 100.0 * gain.at<double>(5) * error.at<double>(5);

          std::cout << "B1 = -lambda * e = " << std::endl << B1 << std::endl;

          // Compare with classic (pseudo-inverse solution)
          cv::Mat A1_inv = A1.inv(cv::DECOMP_SVD);
          cv::Mat classic_soln = A1_inv * B1;
          std::cout << "Classic (J_inv) solution = " << std::endl << classic_soln << std::endl;
          std::cout << "camera velocity = " << std::endl << J * classic_soln << std::endl;

          ////////////////////////////////////////////////////////////////
          // Add constraints to HQP                                     //
          ////////////////////////////////////////////////////////////////

          cascade.addInequality(C1, d1, 0); // Lower angle limits
          cascade.addInequality(C2, d2, 0); // Upper angle limits
          //cascade.addInequality(C5, d5, 0); // Lower speed limits
          //cascade.addInequality(C6, d6, 0); // Upper speed limits
          cascade.addInequality(C3, d3, 0); // Singularity avoidance
          cascade.addEquality(A1, B1, 1); // Main servoing task;

          ////////////////////////////////////////////////////////////////
          // Solve the HQP                                              //
          ////////////////////////////////////////////////////////////////

          cv::Mat q_dot = cv::Mat(6, 1, CV_64F);
          cascade.solveCascade(q_dot);
          std::cout << "[+] Solved cascade" << std::endl;

          ////////////////////////////////////////////////////////////////
          // Send joint velocities to vehicle                           //
          ////////////////////////////////////////////////////////////////

          // Set the finger velocities to zero.
          for(uint i=7; i<13; ++i)
          {
              q_dot.push_back(0.0);
          }

          arm.mInterface->setDOFVelocities(q_dot);

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

          std::cout << "[+] Adaptive gains = " << gain << std::endl;
          std::cout << "[+] Depth (Z) = " << C << std::endl;

          // Print final screw values
          for(int i = 0; i < 7; i++)
          {
              std::cout << "joint velocity[" << i << "] = " << q_dot.at<double>(i) << std::endl;
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
  std::ifstream settings_file( "../settings/test_CArm_HQP_thresh.ini" , std::ifstream::in );
  po::store( po::parse_config_file( settings_file , desc ), vm );
  settings_file.close();
  po::notify( vm );

} // setParameters()

