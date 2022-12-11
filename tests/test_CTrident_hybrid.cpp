/*******************************************
  Title: HOOF (Homography of ORB Features)
  Description: IBVS control of ROV
*******************************************/

//#define DEBUG

#ifdef DEBUG
#define dbg(msg) std::cout << "[DBG] " << __TIME__ << ":" << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": " << msg << std::endl;
#else
#define dbg(msg)
#endif

#include <cmath>
#include <algorithm>
#include <fstream>
#include <deque>

#include "boost/program_options.hpp"
#include "boost/interprocess/managed_shared_memory.hpp"

#include "../../VS/include/utils.hpp"
#include "../../VS/include/CKeyPointMatcher.hpp"
#include "../../VS/include/CROSFrameGrabber.hpp"
#include "../../VS/include/CVehicleInterface.hpp"
#include "../../VS/include/CAlgorithmSwitcher.hpp"
#include "../../VS/include/CImageDecorator.hpp"
#include "../../VS/include/CArm.hpp"
#include "../../VS/include/mouse.hpp"
#include "../../VS/include/CAdaptiveGain.hpp"
#include "../../VS/include/CLMoments.h"

namespace po = boost::program_options;
void setParameters();

int size = 20;
cv::Rect roi;
cv::Rect c_roi;
cv::Rect d_roi;
cv::Mat d_img;
cv::Mat d_img_2;
cv::Point p1;
cv::Point p2;

// -- BEGIN PARAMETERS -- //

// Feature matcher params
double matcherConfidence = 0.98;
double matcherMinEpipolarDist = 1.0;
double matcherRatio = 0.65;

// ROV params
double ROV_VEL_MAX[6];

// Additional velocity screw damping factors
double screw_factor[6];

int roiGrowthRate = 5;

// Threshodling params
int r_min = 200;
int r_max = 255;
int g_min = 0;
int g_max = 255;
int b_min = 0;
int b_max = 255;
// -- END PARAMETERS -- //

int main(int argc, char *argv[])
{
    // Create a ROS node
    ros::init(argc, argv, "test_CRobustMatcher");
    ros::NodeHandle nh;

    // Setup options.
    setParameters();

    // Create track bars for setting RGB range
    cvNamedWindow("thresholding", CV_WINDOW_AUTOSIZE);
    cvCreateTrackbar("r_min","thresholding",&r_min,255);
    cvCreateTrackbar("r_max","thresholding",&r_max,255);
    cvCreateTrackbar("g_min","thresholding",&g_min,255);
    cvCreateTrackbar("g_max","thresholding",&g_max,255);
    cvCreateTrackbar("b_min","thresholding",&b_min,255);
    cvCreateTrackbar("b_max","thresholding",&b_max,255);

    // 0, inf, 0 grad
    vs::CAdaptiveGain lambda_vx(0.0, 0.01, 0.0001);
    vs::CAdaptiveGain lambda_vy(0.0, 0.01, 0.0001);
    vs::CAdaptiveGain lambda_vz(0.0, 100.0, 0.1);

    vs::CAdaptiveGain lambda_wx(0.0, 0.01, 0.00001);
    vs::CAdaptiveGain lambda_wy(0.0, 0.01, 0.00001);
    vs::CAdaptiveGain lambda_wz(0.0, 10.0, 1.0);

    // Get current frame from shared memory
    boost::interprocess::managed_shared_memory msm;

    try
    {
        msm = boost::interprocess::managed_shared_memory(boost::interprocess::open_only, vs::utils::MEMORY_NAME);
    }
    catch(boost::interprocess::interprocess_exception e)
    {
        std::cout << "Exception caught while grabbing frame from shared memory: '"
                  << e.what() << "'" << std::endl;
        std::cout << "Is the frame server subscribed to a valid ros topic?" << std::endl;
    }

    const vs::utils::SharedImageHeader* shared_image_header = msm.find<vs::utils::SharedImageHeader>("MatHeader").first;
    const cv::Mat currentFrame(shared_image_header->size, shared_image_header->type,
                               msm.get_address_from_handle(shared_image_header->handle));

    // Create a decorator
    CImageDecorator dec;
    std::stringstream text;

    // Create an output file
    std::ofstream outputFile("data.csv");
    outputFile << "e_v[0],e_v[1],e_vi[2],e_w[0],e_w[1],e_w[2],v[0],v[1],v[2],v[3],v[4],v[5],v_s[0],v_s[1],v_s[2],v_s[3],v_s[4],v_s[5],c_dist,d_dist,e_dist,e_distance_max,e_distance_ratio,c_orient,d_orient,e_orient,e_orient_max,e_orient_ratio" << std::endl;

    std::ofstream outputFile2("error.csv");
    outputFile2 << "e_x,e_y,e_z,e_roll,e_pitch,e_yaw,e_total" << std::endl;

    // Create a vehicle
    vs::CVehicleInterface trident(nh);
    trident.setNDOF(6);
    cv::Mat startPose;
    trident.getDOFPositions(startPose);

    while(startPose.empty())
    {
        ros::spinOnce();
        trident.getDOFPositions(startPose);
    }

    std::vector<double> currentPose;
    std::vector<double> currentPoseError;
    double currentError = -1.0;

    // Create containers
    cv::Mat c_img;

    //cv::Mat d_img;
    cv::Mat match_img;
    std::vector<cv::KeyPoint> c_keypoints;
    std::vector<cv::KeyPoint> d_keypoints;
    std::vector<cv::DMatch> matches;
    cv::Scalar mcolor(0,255,0);
    cv::Scalar kpcolor(0,0,255);

    // Create KeyPoint matcher
    CKeyPointMatcher rmatcher;
    rmatcher.setAlgorithm("ORB");
    rmatcher.setConfidenceLevel(matcherConfidence); // 0.98
    rmatcher.setMinDistanceToEpipolar(matcherMinEpipolarDist); // 1.0
    rmatcher.setRatio(matcherRatio); // 0.65

    // Create algorithm hierarchy
    CAlgorithmSwitcher<uint, std::string> aswitch("ORB");
    //aswitch.addAlgorithm(50, "SURF");
    aswitch.addAlgorithm(20, "ORB");
    aswitch.addAlgorithm(0, "HARRIS");
    std::string current_alg;

    // Get desired/goal image
    d_img = currentFrame.clone();

    // Extract features from desired image.
    cv::Mat thresh;
    cv::inRange(d_img, cv::Scalar(b_min, g_min, r_min), cv::Scalar(b_max, g_max, r_max), thresh);

    cv::Moments m = cv::moments(thresh, true);
    double d_centroid_x = m.m10 / m.m00;
    double d_centroid_y = m.m01 / m.m00;
    double mu11_prime = (m.m11/m.m00) - (d_centroid_x * d_centroid_y);
    double mu20_prime = (m.m20/m.m00) - (d_centroid_x * d_centroid_x);
    double mu02_prime = (m.m02/m.m00) - (d_centroid_y * d_centroid_y);
    double d_alpha = 0.5 * atan2(2.0 * mu11_prime, mu20_prime - mu02_prime);


    // Display
    cv::namedWindow("sourceImage", CV_WINDOW_AUTOSIZE);
    d_img_2 = d_img.clone();
    cv::imshow("sourceImage", d_img_2);

    // Create event notifier (subject)
    utils::MouseEventNotifier mouse;

    // Create handlers (observers)
    utils::ClicksToROI clicks;

    // Bind handlers to notify
    mouse.m_notifer.connect(boost::bind(&utils::IMouseObserver::notify, &clicks, _1, _2, _3, _4));

    //cvSetMouseCallback("sourceImage", CallbackFunc, &d_img);
    cvSetMouseCallback("sourceImage", utils::mouseCallback, &mouse);

    // Set vehicle home pose
/*
    std::vector<double> home_pose;
    home_pose.push_back(2.5);
    home_pose.push_back(2.0);
    home_pose.push_back(1.9);
    home_pose.push_back(0.0);
    home_pose.push_back(0.0);
    home_pose.push_back(-0.0466);
    trident.setHomePose(home_pose);
*/

    // Drive vehicle to starting pose
    std::cout << "Drive vehicle to start pose then press any key to continue..." << std::endl;
    cv::waitKey();

    std::deque<std::pair<double, double> > d_centroid_log(3);
    std::deque<std::pair<double, double> > c_centroid_log(3);
    std::deque<std::vector<double> > screw_log;


    // Video writer;
    cv::Size vidSize(currentFrame.cols*2, currentFrame.rows);
    int codec = CV_FOURCC('M', 'J', 'P', 'G');
    int frameRate = 24;
    cv::VideoWriter vidWriter("vehicle_HOOF-moment_hydbrid.avi", codec, frameRate, vidSize, true);
    vidWriter.open("video.avi", codec, frameRate, vidSize, true);


    // Loop forever
    while(true)
    {
        // Process ROS callback queue
        ros::spinOnce();

        // Draw ROI on clone of desired image
        d_img_2 = d_img.clone();
        p1 = clicks.getP1();
        p2 = clicks.getP2();
        roi = clicks.getROI();
        cv::rectangle(d_img_2, roi, cv::Scalar(0,0,255), 2, 8, 0);
        cv::imshow("sourceImage", d_img_2);

        // Get current image
        c_img = currentFrame.clone();

        // If the image isn't empty...
        if(c_img.empty()==false)
        {
            ////////////////////////////////////////////////////////////////////
            // Threshold image and extract moments                            //
            ////////////////////////////////////////////////////////////////////
            // Threshold image
            cv::inRange(c_img, cv::Scalar(b_min, g_min, r_min), cv::Scalar(b_max, g_max, r_max), thresh);
            cv::imshow("threshold", thresh);

            // Extract current moments
            m = cv::moments(thresh, true);
            double c_centroid_x = m.m10 / m.m00;
            double c_centroid_y = m.m01 / m.m00;
            mu11_prime = (m.m11/m.m00) - (c_centroid_x * c_centroid_y);
            mu20_prime = (m.m20/m.m00) - (c_centroid_x * c_centroid_x);
            mu02_prime = (m.m02/m.m00) - (c_centroid_y * c_centroid_y);
            double c_alpha = 0.5 * atan2(2.0 * mu11_prime, mu20_prime - mu02_prime);

            ////////////////////////////////////////////////////////////////////
            // Get keypoints and matches                                      //
            ////////////////////////////////////////////////////////////////////

            // Reset keypoints/matches for next iteration
            c_keypoints.clear();
            d_keypoints.clear();
            matches.clear();

            // Get and display matches
            rmatcher.match(c_img, d_img, matches, c_keypoints, d_keypoints, p1, p2);

            // Do we need to switch the algorithm? I.e. did we get enough matches?
            uint nMatches = matches.size();
            current_alg = aswitch.switchAlgorithm(nMatches);
            rmatcher.setAlgorithm(current_alg);

            cv::drawMatches(c_img, c_keypoints, d_img, d_keypoints, matches, match_img, mcolor, kpcolor);

            ////////////////////////////////////////////////////////////////////
            // Start main servoing loop                                       //
            ////////////////////////////////////////////////////////////////////

            // Require at least 4 matches
            if(nMatches > 3)
            {
                double xb[nMatches];
                double yb[nMatches];
                double xa[nMatches];
                double ya[nMatches];

                cv::Mat aHb;
                std::vector<cv::Point2f> srcPoints;
                std::vector<cv::Point2f> dstPoints;

                // Centroid of matched points
                double c_centroid_x = 0.0;
                double c_centroid_y = 0.0;

                double d_centroid_x = 0.0;
                double d_centroid_y = 0.0;

                // Average distance between keypoints
                //double d_distance = 0.0;
                //double c_distance = 0.0;

                // Loop over all matches
                for(uint i=0; i<nMatches; ++i)
                {
                    // Store the co-ordinates of the matching points
                    xa[i] = c_keypoints[matches[i].queryIdx].pt.x;
                    ya[i] = c_keypoints[matches[i].queryIdx].pt.y;
                    xb[i] = d_keypoints[matches[i].trainIdx].pt.x;
                    yb[i] = d_keypoints[matches[i].trainIdx].pt.y;

                    srcPoints.push_back(cv::Point2f(xb[i], yb[i]));
                    dstPoints.push_back(cv::Point2f(xa[i], ya[i]));

                    c_centroid_x += xa[i];
                    c_centroid_y += ya[i];

                    d_centroid_x += xb[i];
                    d_centroid_y += yb[i];

                    /*
                    // Calculate distance between all point pairs
                    for(uint j=0; j<nMatches; ++j)
                    {
                        double c_x2 = c_keypoints[matches[j].queryIdx].pt.x;
                        double c_y2 = c_keypoints[matches[j].queryIdx].pt.y;
                        c_distance += sqrt((xa[i] - c_x2)*(xa[i] - c_x2) + (ya[i] - c_y2)*(ya[i] - c_y2));

                        double d_x2 = d_keypoints[matches[j].trainIdx].pt.x;
                        double d_y2 = d_keypoints[matches[j].trainIdx].pt.y;
                        d_distance += sqrt((xb[i] - d_x2)*(xb[i] - d_x2) + (yb[i] - d_y2)*(yb[i] - d_y2));
                    }
                    */
                }

                /*
                // Compute z-damping based on the difference between the
                // average keypoints seperation in the current and desired images
                d_distance /= double(nMatches * (nMatches-1)) / 2.0 ;
                c_distance /= double(nMatches * (nMatches-1)) / 2.0 ;

                // e = s_d - s_c
                double e_distance = d_distance - c_distance;
                if( (fabs(e_distance) > fabs(e_distance_max)) || (e_distance_max == 0.0) )
                {
                    e_distance_max = e_distance;
                }
                e_distance_ratio = (double(e_distance / e_distance_max));

                */

                ////////////////////////////////////////////////////////////////
                // Compute the orientation of the region of interest          //
                ////////////////////////////////////////////////////////////////

                // Bottom-right point of current image
                double max_xa = *std::max_element(xa,xa+nMatches);
                double max_ya = *std::max_element(ya,ya+nMatches);

                // Top-left point of current image
                double min_xa = *std::min_element(xa,xa+nMatches);
                double min_ya = *std::min_element(ya,ya+nMatches);

                // Bottom-right point of desired image
                double max_xb = *std::max_element(xb,xb+nMatches);
                double max_yb = *std::max_element(yb,yb+nMatches);

                // Top-left point of desired image
                double min_xb = *std::min_element(xb,xb+nMatches);
                double min_yb = *std::min_element(yb,yb+nMatches);

                /*

                // Get orientation of current image
                d_roi = cv::Rect(min_xb, min_yb, (max_xb-min_xb), (max_yb-min_yb));
                c_roi = cv::Rect(min_xa, min_ya, (max_xa-min_xa), (max_ya-min_ya));

                cv::Mat d_roi_img = d_img(d_roi);
                cv::Mat c_roi_img = c_img(c_roi);
                double d_orientation = vs::utils::orientation(d_roi_img);
                double c_orientation = vs::utils::orientation(c_roi_img);

                // Compute yaw-damping based on the orientation difference
                // between the current and desired images.
                // e = s_d - s_c
                double e_orientation = d_orientation - c_orientation;
                if( (fabs(e_orientation) > fabs(e_orientation_max)) || (e_orientation_max == 0.0) )
                {
                    e_orientation_max = e_orientation;
                }
                e_orientation_ratio = double(e_orientation / e_orientation_max);
                */

                ////////////////////////////////////////////////////////////////////
                // Compute centroid as a moving average over the last n iterations //
                ////////////////////////////////////////////////////////////////////

                // Compute centroid
                c_centroid_x /= double(nMatches);
                c_centroid_y /= double(nMatches);

                d_centroid_x /= double(nMatches);
                d_centroid_y /= double(nMatches);

                uint queueSize = 10;

                // If deque = n values, pop oldest.
                if(d_centroid_log.size() == queueSize)
                {
                    d_centroid_log.pop_back();
                    c_centroid_log.pop_back();
                }

                // Add current values.
                d_centroid_log.push_front(std::pair<double, double>(d_centroid_x, d_centroid_y));
                c_centroid_log.push_front(std::pair<double, double>(c_centroid_x, c_centroid_y));

                //BEGIN: Find average over the deque
                double d_x_sum = 0.0;
                double d_y_sum = 0.0;
                double c_x_sum = 0.0;
                double c_y_sum = 0.0;

                // Loop over all deque elements.
                for(uint i = 0; i < d_centroid_log.size(); ++i)
                {
                    d_x_sum += d_centroid_log.at(i).first;
                    d_y_sum += d_centroid_log.at(i).second;
                    c_x_sum += c_centroid_log.at(i).first;
                    c_y_sum += c_centroid_log.at(i).second;
                }

                // Average over n frames
                d_centroid_x = d_x_sum / double(d_centroid_log.size());
                d_centroid_y = d_y_sum / double(d_centroid_log.size());
                c_centroid_x = c_x_sum / double(c_centroid_log.size());
                c_centroid_y = c_y_sum / double(c_centroid_log.size());
                //END: Find avergae over deque

                // Convert centroid to cv::Mat
                std::vector<double> m1;
                m1.push_back(d_centroid_x);
                m1.push_back(d_centroid_y);
                m1.push_back(1.0);
                cv::Mat md(m1);

                std::vector<double> m2;
                m2.push_back(c_centroid_x);
                m2.push_back(c_centroid_y);
                m2.push_back(1.0);
                cv::Mat mc(m2);

                ////////////////////////////////////////////////////////////////
                // Decorate the images we display to the user                 //
                ////////////////////////////////////////////////////////////////

                double image_width = double(c_img.cols);

                // Draw bounding box of matches in current image
                cv::rectangle(match_img, cv::Point2f(min_xa, min_ya), cv::Point2f(max_xa, max_ya), cv::Scalar(255,0,0), 2, 8, 0);

                // Draw bounding box of matches in desired image
                cv::rectangle(match_img, cv::Point2f(min_xb+image_width, min_yb), cv::Point2f(max_xb+image_width, max_yb), cv::Scalar(255,0,0), 2, 8, 0);

                // Draw ROI in desired image
                cv::rectangle(match_img, cv::Point2f(p1.x+image_width, p1.y), cv::Point2f(p2.x+image_width, p2.y), cv::Scalar(0,0,255), 2, 8, 0);

                cv::Point2f centroid(d_centroid_x + image_width, d_centroid_y);
                dec.setImage(match_img);
                dec.addPoint(centroid, "centroid");

                dec.addText("Current image", cv::Point2f(10, 10));
                dec.addText("Reference image", cv::Point2f(10 + image_width, 10));

                text << "Number of matches = " << nMatches;
                dec.addText(text.str(), cv::Point2f(10, 25));
                text.str("");

                text << "Average centroid of matches = (" << d_centroid_x << ", " << d_centroid_y << ")";
                dec.addText(text.str(), cv::Point2f(10, 40));
                text.str("");

                text << "Current algorithm = " << current_alg;
                dec.addText(text.str(), cv::Point2f(10, 55));
                text.str("");

                ////////////////////////////////////////////////////////////////
                // Compute the homography matrix using the matched points     //
                ////////////////////////////////////////////////////////////////

                // aHb is normalized i.e. element (2,2) = 1
                aHb = cv::findHomography(srcPoints, dstPoints, 0, 3);
//                aHb = cv::findHomography(srcPoints, dstPoints, CV_RANSAC, 3);

                std::cout << "Estimated homography aHb: \n" << aHb << std::endl;

                double rho2 = cv::determinant(aHb);

                ////////////////////////////////////////////////////////////////
                // Compute the hybrid interaction matrix                      //
                ////////////////////////////////////////////////////////////////

                double x = c_centroid_x;
                double y = c_centroid_y;

                cv::Mat L_v = cv::Mat::eye(3, 3, CV_64F);
                L_v.mul(-1.0);
                L_v.at<double>(0, 2) = x;
                L_v.at<double>(1, 2) = y;
                L_v.mul(rho2);

                cv::Mat L_v_pinv;
                cv::invert(L_v, L_v_pinv, cv::DECOMP_SVD);

                cv::Mat L_vw = cv::Mat::zeros(3, 3, CV_64F);
                L_vw.at<double>(0, 0) = x*y;
                L_vw.at<double>(0, 1) = -(1+x*x);
                L_vw.at<double>(0, 2) = y;

                L_vw.at<double>(1, 0) = (1+y*y);
                L_vw.at<double>(1, 1) = -x*y;
                L_vw.at<double>(1, 2) = -x;

                L_vw.at<double>(2, 0) = -y;
                L_vw.at<double>(2, 1) = x;
                L_vw.at<double>(2, 2) = 0.0;

                double d = 4.8 - 1.3;

                cv::Mat L_temp_1 = d * L_v_pinv;
                cv::Mat L_temp_2 = d * L_v_pinv * L_vw;
                cv::Mat L_temp_3;
                vs::utils::hconcat(L_temp_1, L_temp_2, L_temp_3);

                cv::Mat L_temp_4 = cv::Mat::zeros(3, 3, CV_64F);
                cv::Mat L_temp_5 = cv::Mat::eye(3, 3, CV_64F);
                cv::Mat L_temp_6;
                vs::utils::hconcat(L_temp_4, L_temp_5, L_temp_6);

                cv::Mat L;
                vs::utils::vconcat(L_temp_3, L_temp_6, L);

                ////////////////////////////////////////////////////////////////
                // Compute the error vector                                   //
                ////////////////////////////////////////////////////////////////

                cv::Mat error = cv::Mat(6, 1, CV_64F);
                error.at<double>(0, 0) = c_centroid_x - d_centroid_x;
                error.at<double>(1, 0) = c_centroid_y - d_centroid_y;
                error.at<double>(2, 0) = std::log(rho2);

                cv::Mat e_w_x = aHb - aHb.t();
                error.at<double>(3, 0) = e_w_x.at<double>(2,1);
                error.at<double>(4, 0) = e_w_x.at<double>(0,2);
                error.at<double>(5, 0) = c_alpha - d_alpha;//e_w_x.at<double>(1,0);

                std::cout << "Error vector: " << std::endl << error << std::endl;

                ////////////////////////////////////////////////////////////////
                // Compute the screw                                          //
                ////////////////////////////////////////////////////////////////

                double velocity_screw[6];
                velocity_screw[0] = -lambda_vx(std::abs(error.at<double>(0, 0))) * error.at<double>(0, 0);
                velocity_screw[1] = -lambda_vy(std::abs(error.at<double>(1, 0))) * error.at<double>(1, 0);
                velocity_screw[2] = lambda_vz(std::abs(error.at<double>(2, 0))) * error.at<double>(2, 0);

                velocity_screw[3] = 0.0; //-lambda_wx(std::abs(e_w.at<double>(0, 0))) * e_w.at<double>(0, 0);
                velocity_screw[4] = 0.0; //-lambda_wy(std::abs(e_w.at<double>(0, 1))) * e_w.at<double>(0, 1);
                velocity_screw[5] = -lambda_wz(std::abs(error.at<double>(5, 0))) * error.at<double>(5, 0);

                ////////////////////////////////////////////////////////////////
                // Saturate velocity screw components                         //
                ////////////////////////////////////////////////////////////////

                // Check each velocity screw component.
                for(uint i = 0; i < 6; ++i)
                {
                    double v_i = fabs(velocity_screw[i]);
                    double v_max_i = fabs(ROV_VEL_MAX[i]);

                    if(v_i > v_max_i)
                    {
                        double scale_i = double(v_max_i) / double(v_i);
                        velocity_screw[i] = velocity_screw[i] * scale_i;
                    } // if()

                    //Check for NaN entries and set them to zero.
                    if(velocity_screw[i] != velocity_screw[i])
                    {
                        velocity_screw[i] = 0.0;
                        std::cout << "NaN detected in velocity screw component " << i << std::endl;
                    }

                } // for()


                // Saturated velocities
                std::cout << velocity_screw[0] << "," << velocity_screw[1] << ","
                           << velocity_screw[2] << "," << velocity_screw[3] << ","
                           << velocity_screw[4] << "," << velocity_screw[5] << ","
                           << std::endl;


                ///////////////////////////////////////////////////////////////////////////
                // Compute velocity screw as a moving average over the last n iterations //
                ///////////////////////////////////////////////////////////////////////////

                std::vector<double> screw;
                screw.push_back(velocity_screw[0]); // V_x
                screw.push_back(velocity_screw[1]); // V_y
                screw.push_back(velocity_screw[2]); // V_z
                screw.push_back(velocity_screw[3]); // Roll
                screw.push_back(velocity_screw[4]); // Pitch
                screw.push_back(velocity_screw[5]); // Yaw

                // Set the arm velocity to zero at the moment.
                for(uint i=6; i<13; ++i)
                {
                    screw.push_back(0.0);
                }

                // If deque = n values, pop oldest.
                if(screw_log.size() == 1)
                {
                    screw_log.pop_back();
                }

                // Add current values.
                screw_log.push_front(screw);

                std::vector<double> averageScrew(6);

                //BEGIN: Find average over the deque
                // Loop over all deque elements.
                dbg("")
                for(uint i = 0; i < screw_log.size(); ++i)
                {
                    for(uint j=0; j<6; ++j)
                    {
                        dbg("i,j =" << i << ", " << j)
                        averageScrew.at(j) += (screw_log.at(i).at(j) / double(screw_log.size()));
                    }
                }

                ////////////////////////////////////////////////////////////////
                // Send velocity screw to vehicle                             //
                ////////////////////////////////////////////////////////////////

                trident.setDOFVelocities(cv::Mat(averageScrew));

                // Display results
                dec.display();
                cv::waitKey(30);

            } // if num keypoints > 3

            ////////////////////////////////////////////////////////////////////
            // If not enough keypoints in the ROI then grow the ROI.          //
            ////////////////////////////////////////////////////////////////////
            else
            {
                p1.x = (p1.x - roiGrowthRate)>0 ? (p1.x - roiGrowthRate) : p1.x;
                p1.y = (p1.y - roiGrowthRate)>0 ? (p1.y - roiGrowthRate) : p1.y;

                p2.x = (p2.x + roiGrowthRate)<d_img.cols ? (p2.x + roiGrowthRate) : p2.x;
                p2.y = (p2.y + roiGrowthRate)<d_img.rows ? (p2.y + roiGrowthRate) : p2.y;

                roi = cv::Rect(p1.x, p1.y, (p2.x-p1.x), (p2.y-p1.y));

                dbg("p1.x = " << p1.x)
                dbg("p1.y = " << p1.y)
                dbg("p2.x = " << p2.x)
                dbg("p2.y = " << p2.y)

                dbg("WARNING < 3 MATCHES! (" << nMatches << ")")
                cv::waitKey(30);
            }


            // Write frame to video
            vidWriter.write(dec.getImage());


        } // if images is not empty


    } // while(true)

    outputFile.close();
    outputFile2.close();

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

    // Feature matcher params
    ("matcher.confidence", po::value< double >( &matcherConfidence )->required(), "matcherConfidence")
    ("matcher.minEpipolarDist", po::value< double >( &matcherMinEpipolarDist )->required(), "matcherMinEpipolarDist")
    ("matcher.ratio", po::value< double >( &matcherRatio )->required(), "matcherRatio")

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

    // Misc params
    ("misc.roiGrowthRate", po::value< int >( &roiGrowthRate )->required(), "roiGrowthRate")

    ;

    // Load setting file.
    po::variables_map vm;
    std::ifstream settings_file( "../settings/trident_settings.ini" , std::ifstream::in );
    po::store( po::parse_config_file( settings_file , desc ), vm );
    settings_file.close();
    po::notify( vm );

} // setParameters()
