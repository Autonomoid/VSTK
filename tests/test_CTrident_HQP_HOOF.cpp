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
#include "../../VS/include/CHQPSolver.hpp"

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

// -- END PARAMETERS -- //



int main(int argc, char *argv[])
{
    // Create a ROS node
    ros::init(argc, argv, "test_CRobustMatcher");
    ros::NodeHandle nh;

    // Setup options.
    setParameters();

    // Disturbance / current terms
    int dist_x = 0.0;
    int dist_y = 0.0;
    int dist_z = 0.0;
    cvNamedWindow("Disturbance", CV_WINDOW_AUTOSIZE);
    cvCreateTrackbar("x (cm/s)", "Disturbance", &dist_x, 200);
    cvCreateTrackbar("y (cm/s)", "Disturbance", &dist_y, 200);
    cvCreateTrackbar("z (cm/s)", "Disturbance", &dist_z, 200);

    // 0, inf, 0 grad
//    vs::CAdaptiveGain lambda_vx(1.666, 0.166, 1.666);

    // These values give a lambda proportional to the error.
    vs::CAdaptiveGain lambda_vx(0.0, 0.01, 0.00001);
    vs::CAdaptiveGain lambda_vy(0.0, 0.01, 0.00001);
    vs::CAdaptiveGain lambda_vz(0.0, 10.0, 0.1);

    vs::CAdaptiveGain lambda_wx(0.0, 0.01, 0.00001);
    vs::CAdaptiveGain lambda_wy(0.0, 0.01, 0.00001);
    vs::CAdaptiveGain lambda_wz(0.0, 0.01, 0.00001);

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
    std::time_t t = time(0);
    struct std::tm* now = localtime(&t);
    char buffer [80];
    std::strftime(buffer,80,"../results/trident_HOOF_HQP/%Y-%m-%d_%H:%M:%S.csv", now);

    std::ofstream outputFile(buffer);

    outputFile << "error x, error y, error z, error roll, error pitch, error yaw,"
               << "velocity x, velocity y, velocity z, velocity roll, velocity pitch, velocity yaw,"
               << "gain x, gain y, gain z, gain roll, gain pitch, gain yaw,"
               << "x, y, z, roll, pitch, yaw,"
               << std::endl;

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

    cv::Mat currentPose;
    cv::Mat currentPoseError;

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



    //Get desired/goal image
    d_img = currentFrame.clone();

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

    std::deque<std::pair<double, double> > centroid_log(3);
    std::deque<std::vector<double> > screw_log;

    /*
    // Video writer;
    cv::Size vidSize(currentFrame.cols*2, currentFrame.rows);
    int codec = CV_FOURCC('M', 'J', 'P', 'G');
    int frameRate = 24;
    cv::VideoWriter vidWriter("video.avi", codec, frameRate, vidSize, true);
    vidWriter.open("video.avi", codec, frameRate, vidSize, true);
    */

    cv::Mat e_v, e_w;
    double rho2 = 0.0;
    cv::Mat solution;

    double x_gain = 0.0;
    double y_gain = 0.0;
    double z_gain = 0.0;
    double roll_gain = 0.0;
    double pitch_gain = 0.0;
    double yaw_gain = 0.0;

    // Loop forever
    int iterations = 1;
    //while(iterations < 101)

    while(true)
    {
        std::cout << "Iteration " << iterations << " / " << 100 << std::endl;

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
                double centroid_x = 0.0;
                double centroid_y = 0.0;

                // Average distance between keypoints
                double d_distance = 0.0;
                double c_distance = 0.0;

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

                    centroid_x += xb[i];
                    centroid_y += yb[i];

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
                // Compute centoid as a moving average over the last n iterations //
                ////////////////////////////////////////////////////////////////////

                // Compute centroid
                centroid_x /= double(nMatches);
                centroid_y /= double(nMatches);

                uint queueSize = 10;

                // If deque = n values, pop oldest.
                if(centroid_log.size() == queueSize)
                {
                    centroid_log.pop_back();
                }

                // Add current values.
                centroid_log.push_front(std::pair<double, double>(centroid_x, centroid_y));

                //BEGIN: Find average over the deque
                double x_sum = 0.0;
                double y_sum = 0.0;

                // Loop over all deque elements.
                for(uint i = 0; i < centroid_log.size(); ++i)
                {
                    x_sum += centroid_log.at(i).first;
                    y_sum += centroid_log.at(i).second;
                }

                // Average over n frames
                centroid_x = x_sum / double(centroid_log.size());
                centroid_y = y_sum / double(centroid_log.size());
                //END: Find avergae over deque

                // Convert centroid to cv::Mat
                std::vector<double> m1;
                m1.push_back(centroid_x);
                m1.push_back(centroid_y);
                m1.push_back(1.0);
                cv::Mat m(m1);

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

                cv::Point2f centroid(centroid_x + image_width, centroid_y);
                dec.setImage(match_img);
                dec.addPoint(centroid, "centroid");

                dec.addText("Current image", cv::Point2f(10, 10));
                dec.addText("Reference image", cv::Point2f(10 + image_width, 10));

                text << "Number of matches = " << nMatches;
                dec.addText(text.str(), cv::Point2f(10, 25));
                text.str("");

                text << "Average centroid of matches = (" << centroid_x << ", " << centroid_y << ")";
                dec.addText(text.str(), cv::Point2f(10, 40));
                text.str("");

                text << "Current algorithm = " << current_alg;
                dec.addText(text.str(), cv::Point2f(10, 55));
                text.str("");

                ////////////////////////////////////////////////////////////////
                // Compute the homography matrix using the matched points     //
                ////////////////////////////////////////////////////////////////

                aHb = cv::findHomography(srcPoints, dstPoints, CV_RANSAC, 3);

                aHb /= aHb.at<double>(2,2); // Apply a scale factor to have aHb[2][2] = 1
                std::cout << "Estimated homography aHb: \n" << aHb << std::endl;

                rho2 = cv::determinant(aHb);

                ////////////////////////////////////////////////////////////////
                // Compute velocity screw using the homography matrix         //
                ////////////////////////////////////////////////////////////////

                cv::Mat I = cv::Mat::eye(3, 3, CV_64F);

                dbg("")
                // Compute translational error.
                e_v = (aHb - I) * m;

                dbg("")
                // Compute [e_w]_x from which the rotational error may be extracted.
                cv::Mat e_w_x = aHb - aHb.t();
                std::vector<double> e_w_1;
                e_w_1.push_back(e_w_x.at<double>(2,1));
                e_w_1.push_back(e_w_x.at<double>(0,2));
                e_w_1.push_back(e_w_x.at<double>(1,0));
                e_w = cv::Mat(e_w_1);

                dbg("")
                // Compute the raw translational and rotational velocity screw components.
                e_w.at<double>(0, 0) *= 1.0;
                e_w.at<double>(0, 1) *= 1.0;
                e_w.at<double>(0, 2) *= 300.0;

                ////////////////////////////////////////////////////////////////////////////
                // Build HQP                                                              //
                ////////////////////////////////////////////////////////////////////////////

                // Add constraints to HQP object.
                vs::CHQPSolver cascade;

                // Set the regularization term.
                cascade.setEpsilon(1E-6);

                //// Set the limits on the vehicle velocity

                // Lower limits, q_i > -0.1
                cv::Mat C1 = -1 * cv::Mat::eye(6, 6, CV_64F);
                cv::Mat d1 = cv::Mat::zeros(6, 1, CV_64F);
                d1.at<double>(0) = -0.1;
                d1.at<double>(1) = -0.1;
                d1.at<double>(2) = -0.1;
                d1.at<double>(3) = -0.1;
                d1.at<double>(4) = -0.1;
                d1.at<double>(5) = -0.1;

                // Upper limits, q_i < 0.1
                cv::Mat C2 = cv::Mat::eye(6, 6, CV_64F);
                cv::Mat d2 = cv::Mat::zeros(6, 1, CV_64F);
                d2.at<double>(0) = 0.1;
                d2.at<double>(1) = 0.1;
                d2.at<double>(2) = 0.1;
                d2.at<double>(3) = 0.1;
                d2.at<double>(4) = 0.1;
                d2.at<double>(5) = 0.1;

                //cascade.addInequality(C1, d1, 0); // Lower limits
                //cascade.addInequality(C2, d2, 0); // Upper limits


                //// Main servoing task.
                cv::Mat A1 = cv::Mat::eye(6, 6, CV_64F);
                A1.at<double>(4,0) = 0.0;
                A1.at<double>(5,0) = 0.0;

                x_gain = lambda_vx(std::abs(e_v.at<double>(0)));
                y_gain = lambda_vy(std::abs(e_v.at<double>(1)));
                z_gain = lambda_vz(std::abs(log(rho2)));
                yaw_gain = lambda_wz(std::abs(e_w.at<double>(2)));

                cv::Mat B1 = cv::Mat::zeros(6, 1, CV_64F);
                B1.at<double>(0,0) = -x_gain * e_v.at<double>(0);
                B1.at<double>(1,0) = -y_gain * e_v.at<double>(1);
                B1.at<double>(2,0) = z_gain * log(rho2);
                B1.at<double>(3,0) = 0.0; //-lambda_wx(std::abs(e_w.at<double>(0, 0))) * e_w.at<double>(0, 0);
                B1.at<double>(4,0) = 0.0; //-lambda_wy(std::abs(e_w.at<double>(0, 1))) * e_w.at<double>(0, 1);
                B1.at<double>(5,0) = -yaw_gain * e_w.at<double>(02);

                cascade.addEquality(A1, B1, 1); // Main servoing task;

                ////////////////////////////////////////////////////////////////
                // Send velocity screw to vehicle                             //
                ////////////////////////////////////////////////////////////////

                cascade.solveCascade(solution);

                cv::Mat screw;

                std::vector<double> disturbance(6, 0.0);
                disturbance.at(0) = 0.01 * dist_x;
                disturbance.at(1) = 0.01 * dist_y;
                disturbance.at(2) = 0.01 * dist_z;

                for(int i=0; i<solution.rows; ++i)
                {
                    screw.push_back(solution.at<double>(i) + disturbance.at(i));
                }

                trident.setDOFVelocities(screw);

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

            /*
            // Write frame to video
            vidWriter.write(dec.getImage());
            */

        } // if images is not empty

        trident.getDOFPositions(currentPose);

        outputFile << e_v.at<double>(0) << "," << e_v.at<double>(1) << ","
                   << log(rho2) << "," << e_w.at<double>(0) << ","
                   << e_w.at<double>(1) << "," << e_w.at<double>(2) << ","
                   << solution.at<double>(0) << "," << solution.at<double>(1) << ","
                   << solution.at<double>(2) << "," << solution.at<double>(3) << ","
                   << solution.at<double>(4) << "," << solution.at<double>(5) << ","
                   << x_gain << "," << y_gain << "," << z_gain << ","
                   << roll_gain << "," << pitch_gain << "," << yaw_gain << ","
                   << currentPose.at<double>(0) << "," << currentPose.at<double>(1) << ","
                   << currentPose.at<double>(2) << "," << currentPose.at<double>(3) << ","
                   << currentPose.at<double>(4) << "," << currentPose.at<double>(5) << ","
                   << std::endl;

        ++iterations;
    } // while(true)

    outputFile.close();

    std::cout << "Finished all iterations - exiting normally." << std::endl;

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
    std::ifstream settings_file( "../settings/test_CTrident_HQP_HOOF.ini" , std::ifstream::in );
    po::store( po::parse_config_file( settings_file , desc ), vm );
    settings_file.close();
    po::notify( vm );

} // setParameters()
