#include <omp.h>
#include "../include/utils.hpp"

//#define DEBUG
#ifdef DEBUG
#define dbg(msg) std::cout << "[DBG] " << __TIME__ << ":" << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": " << msg << std::endl;
#else
#define dbg(msg)
#endif

namespace vs
{
namespace utils
{

const char* MEMORY_NAME = "sharedFrame";


/**
 * @brief
 *
 * @param p
 * @param u0
 * @param v0
 * @param fx
 * @param fy
 * @param px
 * @param py
 *
 * @return
 */
cv::Point2f
meterToPixel(cv::Point2f p, double u0, double v0, double fx, double fy, double px, double py)
{
    dbg("")

    // (u0, v0) -- image origin
    // fx. fy -- focal lengths
    // px, py -- pixel width, pixel height

    double pixelRatio_x = fx / px;
    double pixelRatio_y = fy / py;
    double u = u0 + (p.x * pixelRatio_x);
    double v = v0 + (p.y * pixelRatio_y);
    return cv::Point2f(u, v);
}


/**
 * @brief
 *
 * @param p
 * @param u0
 * @param v0
 * @param fx
 * @param fy
 * @param px
 * @param py
 *
 * @return
 */
cv::Point2f
pixelToMeter(cv::Point2f p, double u0, double v0, double fx, double fy, double px, double py)
{
    dbg("")

    // (u0, v0) -- image origin
    // fx. fy -- focal lengths
    // px, py -- pixel width, pixel height

    double pixelRatio_x = fx / px;
    double pixelRatio_y = fy / py;

    double x = (p.x - u0) / pixelRatio_x;
    double y = (p.y - v0) / pixelRatio_y;

    return cv::Point2f(x, y);
}


/**
 * @brief
 *
 * @param p
 * @param k
 *
 * @return
 */
cv::Point2f
addRadialDistortion(cv::Point2f p, double k)
{
    dbg("")
    double x = p.x;
    double y = p.y;
    double r_squared = x*x + y*y;
    double L = 1 + (k * r_squared);
    double x_d = x * L;
    double y_d = y * L;
    return cv::Point2f(x_d, y_d);
}


/**
 * @brief
 *
 * @param rad
 *
 * @return
 */
double
radToDeg(double rad)
{
    return ((rad/3.14159265358979323) * 180.0);
}


/**
 * @brief
 *
 * @param deg
 *
 * @return
 */
double
degToRad(double deg)
{
    return ((deg * 3.14159265358979323) / 180.0);
}


/**
 * @brief
 *
 * @param worldPoints
 * @param imagePoints
 */
void
worldToImage(std::vector<cv::Point2f>& worldPoints, std::vector<cv::Point2f>& imagePoints,
                     double fx, double fy, double px, double py, double u0, double v0, double k)
{
    //Create desired image features from the points
    for(std::vector<cv::Point2f>::iterator it = worldPoints.begin(); it != worldPoints.end(); ++it)
    {
        // Add radial distortion
        cv::Point2f temp = addRadialDistortion((*it), k);
        dbg("temp.x = " << temp.x << ", temp.y = " << temp.y)

        // Convert from meters to pixels
        cv::Point2f temp_px = meterToPixel(temp, u0, v0, fx, fy, px, py);
        dbg("temp_px.x = " << temp_px.x << ", temp_px.y = " << temp_px.y)

        // Add to desiredFeatures vector
        imagePoints.push_back(temp_px);
    }
}


/**
 * @brief
 *
 * @param R
 *
 * @return
 */
std::vector<double>
rotationMatToVec(cv::Mat& R)
{
    dbg("")

    // Sanity check
    if((R.cols != 3) || (R.rows != 3))
    {
        throw "ERROR -- matrix dimensions incorrect!";
    }

    dbg("")

    // Rotation vector
    std::vector<double> r;

    // 2*sin(theta) = sqrt( (R10-R01)^2 + (R20-R00)^2 + (R21-R12)^2) )
    double dR_10 = R.at<double>(1,0)-R.at<double>(0,1);
    double dR_21 = R.at<double>(2,1)-R.at<double>(1,2);
    double dR_20 = R.at<double>(2,0)-R.at<double>(0,2);
    double s = 0.5 * sqrt((dR_10 * dR_10) + (dR_20 * dR_20) + (dR_21 * dR_21));

    // cos(theta) = (Tr(R) - 1) / 2.0
    double c = (R.at<double>(0,0)+R.at<double>(1,1)+R.at<double>(2,2)-1)/2.0;
    double theta = atan2(s, c);
    double minimum = 0.001;

    dbg("")

    // If theta is not close to zero
    if(theta > minimum)
    {
        dbg("")

        // If sin(theta) is not close to zero
        if(s > minimum)
        {
            dbg("")

            r.push_back(theta * (dR_21/(2*s)));
            r.push_back(theta * (-dR_20/(2*s)));
            r.push_back(theta * (dR_10/(2*s)));
        }
        else
        {
            r.push_back(sqrt((R.at<double>(0,0)-c)/(1-c)));
            if(dR_21 < 0)
            {
                dbg("")
                r.at(0) = -r.at(0);
            }
            r.at(0) *= theta;

            r.push_back(sqrt((R.at<double>(1,1)-c)/(1-c)));
            if(-dR_20 < 0)
            {
                dbg("")
                r.at(1) = -r.at(1);
            }
            r.at(1) *= theta;

            r.push_back(sqrt((R.at<double>(2,2)-c)/(1-c)));
            if(dR_10 < 0)
            {
                dbg("")
                r.at(2) = -r.at(2);
            }
            r.at(2) *= theta;
        }
    }
    else // if theta is almost zero then set to zero.
    {
        r.push_back(0.0);
        r.push_back(0.0);
        r.push_back(0.0);
    }

    return r;
}


/**
 * @brief
 *
 * @param v
 * @param R
 */
void
VecToRotationMat(std::vector<double>& v, cv::Mat& R)
{
    R = cv::Mat::zeros(3, 3, CV_64F);
    double cx = cos(v.at(0));
    double cy = cos(v.at(1));
    double cz = cos(v.at(2));
    double sx = sin(v.at(0));
    double sy = sin(v.at(1));
    double sz = sin(v.at(2));

    R.at<double>(0, 0) = cy*cz;
    R.at<double>(0, 1) = cz*sx*sy - cx*sz;
    R.at<double>(0, 2) = cx*cz*sy + sx*sz;

    R.at<double>(1, 0) = cy*sz;
    R.at<double>(1, 1) = cx*cz + sx*sy*sz;
    R.at<double>(1, 2) = -cz*sx + cx*sy*sz;

    R.at<double>(2, 0) = -sy;
    R.at<double>(2, 1) = cy*sx;
    R.at<double>(2, 2) = cx*cy;
}


/**
 * @brief
 *
 * @param H
 * @param R
 */
void
HTransformToRotationMatrix(const cv::Mat& H, cv::Mat& R)
{
    for(unsigned int i=0; i<3; ++i)
    {
        for(unsigned int j=0; j<3; ++j)
        {
            R.at<double>(i, j) = H.at<double>(i, j);
        }
    }
}


/**
 * @brief rotationMatToAxisAngle
 * @param R
 * @param theta
 * @param u
 */
void
rotationMatToAxisAngle(cv::Mat& R, double theta, cv::Mat& u)
{
    double c = 0.5*(R.at<double>(0,0) + R.at<double>(1,1) + R.at<double>(2,2) - 1);

    theta = acos(c);

    u.at<double>(0,0) = (R.at<double>(2,1) - R.at<double>(1,2))/(2.0*sin(theta));
    u.at<double>(0,1) = (R.at<double>(0,2) - R.at<double>(2,0))/(2.0*sin(theta));
    u.at<double>(0,2) = (R.at<double>(1,0) - R.at<double>(0,1))/(2.0*sin(theta));
}


/**
 * @brief
 *
 * @param v
 * @param M
 */
void
PoseToRotationMatrix(cv::Mat& pose, cv::Mat& M)
{
    // Sanity checks
    if(pose.rows < 6)
    {
        dbg("ERROR: Vector must be of length 6.")
        throw "ERROR: Vector must be of length 6.";
    }

    double u_x = pose.at<double>(3);
    double u_y = pose.at<double>(4);
    double u_z = pose.at<double>(5);

    double theta = sqrt(u_x*u_x + u_y*u_y + u_z*u_z);
    double epsilon = 0.0001;

    // If theta is not close to zero...
    if(theta > epsilon)
    {
        double s = sin(theta);
        double c = cos(theta);
        double c1 = 1 - c;

        M = cv::Mat::zeros(3, 3, CV_64F);

        M.at<double>(0, 0) = c + u_x*u_x*c1;
        M.at<double>(0, 1) = u_x*u_y*c1 - u_z*s;
        M.at<double>(0, 2) = u_x*u_z*c1 + u_y*s;

        M.at<double>(1, 0) = u_y*u_x*c1 + u_z*s;
        M.at<double>(1, 1) = c + u_y*u_y*c1;
        M.at<double>(1, 2) = u_y*u_z*c1 - u_x*s;

        M.at<double>(2, 0) = u_z*u_x*c1 - u_y*s;
        M.at<double>(2, 1) = u_z*u_y*c1 + u_x*s;
        M.at<double>(2, 2) = c + u_z*u_z*c1;
    }
    else
        // Assume theta equals zero and set M to the identity matrix.
    {
        for(int i = 0; i < 3; i++)
        {
            for(int j = 0; j < 3; j++)
            {
                M.at<double>(i, i) = 1.0;
                M.at<double>(i, j) = 0.0;
            }
        }
    }
}


/**
 * @brief
 *
 * @param pose
 * @param H
 */
void
PoseToHTransform(cv::Mat& pose, cv::Mat& H)
{
    // Sanity checks
    if(pose.rows < 6)
    {
        dbg("ERROR: Pose requires 6 rows")
        throw "ERROR: Pose requires 6 rows";
    }

    H = cv::Mat::zeros(4, 4, CV_64F);
    H.at<double>(3, 3) = 1.0;

    // Convert theta-u component to homogeneous matrix.
    cv::Mat r(3, 3, CV_64F);
    PoseToRotationMatrix(pose, r);

    for(int i = 0; i < 3; i++)
    {
        // Set translational component.
        H.at<double>(i, 3) = pose.at<double>(i);

        // Set rotational component.
        for(int j = 0; j < 3; j++)
        {
            H.at<double>(i, j) = r.at<double>(i, j);
        }
    }
}


/**
 * @brief
 *
 * @param H
 * @param pose
 */
void
HTransformToPose(const cv::Mat& H, cv::Mat& pose)
{
    pose = cv::Mat(6, 1, CV_64F);
    dbg("")

    // Sanity check
    if(H.rows !=4 || H.cols != 4)
    {
        dbg("ERROR - Matrix H has incorrect dimensions")
        throw "ERROR - Matrix H has incorrect dimensions";
    }

    dbg("")

    // Extract rotation matrix from H.
    cv::Mat R(3, 3, CV_64F);
    for(uint i=0; i<3; ++i)
    {
        for(uint j=0; j<3; ++j)
        {
            R.at<double>(i, j) = H.at<double>(i, j);
            dbg("i=" << i << ", j=" << j)
        }
    }

    dbg("")

    // Convert the rotation matrix to a theta-U (tu) vector.
    std::vector<double> tu = rotationMatToVec(R);

    dbg("")

    // Concatenate the translation component of H with tu.
    for(uint i=0; i<3; ++i)
    {
        pose.at<double>(i) = H.at<double>(i, 3);
        pose.at<double>(3 + i) = tu.at(i);
    }
}


/**
 * @brief combineHTransforms
 * @param A
 * @param B
 * @param C
 */
void
combineHTransforms(const cv::Mat& A, const cv::Mat& B, cv::Mat& C)
{
    C = cv::Mat::zeros(4, 4, A.type());
    C.at<double>(3, 3) = 1.0;

    cv::Mat A_rot = cv::Mat::zeros(3, 3, A.type());
    vs::utils::HTransformToRotationMatrix(A, A_rot);

    cv::Mat B_rot = cv::Mat::zeros(3, 3, B.type());
    vs::utils::HTransformToRotationMatrix(B, B_rot);

    cv::Mat C_rot;
    C_rot = A_rot * B_rot;

    cv::Mat C_trans = cv::Mat::zeros(3, 1, A.type());
    cv::Mat A_trans = cv::Mat::zeros(3, 1, A.type());
    cv::Mat B_trans = cv::Mat::zeros(3, 1, A.type());
    for(int i=0; i<3; ++i)
    {
        A_trans.at<double>(i) = A.at<double>(i, 3);
        B_trans.at<double>(i) = B.at<double>(i, 3);
    }

    C_trans = A_rot * B_trans + A_trans;

    for(int i = 0; i < 3; i++)
    {
        // Set translational component.
        C.at<double>(i, 3) = C_trans.at<double>(i);

        // Set rotational component.
        for(int j = 0; j < 3; j++)
        {
            C.at<double>(i, j) = C_rot.at<double>(i, j);
        }
    }

}


/**
 * @brief
 *
 * @param v
 * @param M
 */
void
ThreeVectorToSkewMatrix(std::vector<double>& v, cv::Mat& M)
{
    // Sanity check
    if(v.size() < 3)
    {
        dbg("ERROR: vector too short!")
    }

    M = cv::Mat::zeros(3, 3, CV_64F);
    M.at<double>(1,0) = v.at(2);
    M.at<double>(0,1) = -1.0 * v.at(2);

    M.at<double>(0,2) = v.at(1);
    M.at<double>(2,0) = -1.0 * v.at(1);

    M.at<double>(2,1) = v.at(0);
    M.at<double>(1,2) = -1.0 * v.at(0);
}


/**
 * @brief
 *
 * @param H
 * @param twist
 */
void
HTransformToTwist(cv::Mat& H, cv::Mat& twist)
{
    // Sanity checks
    // twist matrix is 6x6

    // Check definition of twist = {R, R[T]_x; 0; R]

    // Extract translation component from H
    std::vector<double> t;
    t.push_back(H.at<double>(0,3));
    t.push_back(H.at<double>(1,3));
    t.push_back(H.at<double>(2,3));

    // Convert it to a skew matrix
    cv::Mat skew(3, 3, CV_64F);
    ThreeVectorToSkewMatrix(t, skew);

    cv::Mat R = cv::Mat(3, 3, CV_64F);
    vs::utils::HTransformToRotationMatrix(H, R);
    cv::Mat RT = R * skew;

    // Map components to M
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            twist.at<double>(i, j) = H.at<double>(i, j);
            twist.at<double>(i+3, j+3) = H.at<double>(i, j);
            twist.at<double>(i, j+3) = RT.at<double>(i, j);
        }
    }
}


/**
 * @brief
 *
 * @param pose
 * @param twist
 */
void
PoseToTwist(cv::Mat& pose, cv::Mat& twist)
{
    cv::Mat H(3, 4, CV_64F);
    PoseToHTransform(pose, H);
    HTransformToTwist(H, twist);
}


/**
 * @brief
 *
 * @param a
 * @param b
 *
 * @return
 */
std::vector<double>
vectorDiff(std::vector<double>& a, std::vector<double>& b)
{
    // Sanity check
    if(a.size() != b.size())
    {
        throw "Cannot perform vectorDiff. Vectors have different lengths!";
    }

    std::vector<double> c;
    std::vector<double>::iterator it_a = a.begin();
    std::vector<double>::iterator it_b = b.begin();

    for(; it_a != a.end() && it_b != b.end(); ++it_a, ++it_b)
    {
        c.push_back(it_a - it_b);
    }

    return c;
}


/**
 * @brief
 *
 * @param approachInput
 * @param interventionInput
 * @param criterion
 * @param approachThreshold
 * @param interventionThreshold
 *
 * @return
 */
double
distributeControl(double approachInput, double interventionInput, double criterion,
                          double approachThreshold, double interventionThreshold)
{
    // Approach phase -- only use vehicle
    if(criterion > approachThreshold)
    {
        return approachInput;
    }

    // Intervention phase -- only use arm
    else if(criterion < interventionThreshold)
    {
        return interventionInput;
    }

    // In-between phase -- use combination of vehicle and arm.
    else
    {
        return (
                   (criterion - interventionThreshold)*approachInput +
                   (approachThreshold - criterion)*interventionInput
               ) /
               (approachThreshold - interventionThreshold);
    }
}


/**
 * @brief
 *
 * @param input
 *
 * @return
 */
double
FourierSineSynth(double time, std::vector<std::pair<double,double> >& input)
{
    double output = 0.0;
    double frequency = 0.0;
    double amplitude = 0.0;

    std::vector<std::pair<double,double> >::iterator iter = input.begin();

    for(; iter!=input.end(); ++iter)
    {
        amplitude = (*iter).first;
        frequency = (*iter).second;
        output += amplitude * sin(2 * 3.1415926535 * frequency * time);
    }

    return output;
}


/**
 * @brief
 *
 * @param img
 * @param percentage
 */
void
saltAndPepperNoise(cv::Mat& img, int percentage)
{
    uchar max = uchar((200 - percentage) / percentage);
    cv::Mat noise(img.size(), CV_8U);
    cv::randu(noise, cv::Scalar(0), cv::Scalar(max+1));
    img.setTo(cv::Scalar(0,0,0), noise==0);
    img.setTo(cv::Scalar(255,255,255), noise==(max-1));
}


/**
 * @brief
 *
 * @param img
 * @param deviation
 */
void
gaussianNoise(cv::Mat& img, int deviation)
{
    cv::Mat noise = cv::Mat(img.size(), img.type());
    cv::RNG rng(cv::getTickCount());
    rng.fill(noise, cv::RNG::NORMAL, 0, deviation); // bottleneck
    img = img + noise;
}


/**
 * @brief Add simmulated illumination conditions to image
 *
 * @param input
 * @param output
 * @param mode
 * @param theta_x
 * @param theta_y
 * @param targetDistance
 * @param beamBrightness_coeff
 * @param beamBrightness_exponent
 * @param beamCenterX
 * @param beamCenterY
 * @param beamHeight
 * @param beamWidth
 *
 * Mode 0 --> Gaussian beam profile.
 * Mode 1 --> 1/r beam profile.
 * Mode 2 --> UNiform beam profile
 *
 * targetDistance --> 1/r^2 intensity fall-off
 *
 * Beam brightness --> beambrightness_coeff * (10^beambrightness_exponent)
 *
 * theta_x and theta_y --> Angle between beam and target surface.
 */
void
illuminate(const cv::Mat& input, cv::Mat& output, int mode, double theta_x,
                   double theta_y, double targetDistance, int beamBrightness_coeff,
                   int beamBrightness_exponent, int beamCenterX, int beamCenterY,
                   int beamHeight, int beamWidth)
{
    double inBeam = 1.0;
    double r = 0.0;
    double theta = 0.0;
    double rmax = input.rows / 2.0;
    double A = 1.0/(2.0 * M_PI * (double)beamWidth * (double)beamHeight);
    double B, C, D, E = 0.0;
    double beamProfile = 0.0;
    double beamBrightness = 0.0;
    double distanceAttenuation = 0.0;
    double illumination = 1.0;

    // Choose beam profile
    switch(mode)
    {
    case 0: // Gaussian beam
    {
        // Loop over all pixels
        for(double y=0; y<input.rows-1; ++y)
        {
            for(double x=0; x<input.cols-1; ++x)
            {
                B = 1.0 / (cos(theta_y) * (double)beamWidth * (double)beamWidth);
                C = 1.0 / (cos(theta_x) * (double)beamHeight * (double)beamHeight);
                D = -(B*(x-beamCenterX)*(x-beamCenterX));
                E = -(C*(y-beamCenterY)*(y-beamCenterY));

                beamProfile = A * exp(D + E);
                inBeam = 1.0;

                beamBrightness = beamBrightness_coeff * pow(10, beamBrightness_exponent);
                distanceAttenuation = 1.0 /(targetDistance * targetDistance);
                illumination = inBeam * beamBrightness * beamProfile * distanceAttenuation;

                // Apply illumination correction to all channels
                for(int c = 0; c < 3; c++)
                {
                    output.at<cv::Vec3b>(y, x)[c] = cv::saturate_cast<uchar>(input.at<cv::Vec3b>(y, x)[c] *
                                                    illumination);
                }
            }
        }
        break;
    } // case 0

    case 1: // 1/r beam
    {
        // Loop over all pixels
        for(double y=0; y<input.rows-1; ++y)
        {
            for(double x=0; x<input.cols-1; ++x)
            {
                // Circular 1/r
                //r = sqrt((x-beamCenterX)*(x-beamCenterX) + (y-beamCenterY)*(y-beamCenterY));
                //beamProfile = (r/rmax) <= 1.0 ? 1-(r/rmax) : 1.0;

                // Ellipsoidal 1/r
                r = sqrt((x-beamCenterX)*(x-beamCenterX) + (y-beamCenterY)*(y-beamCenterY));

                theta = atan2(y-beamCenterY, x-beamCenterX);

                rmax = (beamWidth * beamHeight) / sqrt((beamHeight * cos(theta))*(beamHeight * cos(theta))
                                                       + (beamWidth * sin(theta))*(beamWidth * sin(theta)));

                beamProfile = (r/rmax) <= 1.0 ? 1-(r/rmax) : 0.0;
                inBeam = ((((x-beamCenterX) * (x-beamCenterX) / (beamWidth * beamWidth)) + ((y-beamCenterY) * (y-beamCenterY) / (beamHeight * beamHeight)))<= 1) ? 1.0 : 0.0;

                beamBrightness = beamBrightness_coeff * pow(10, beamBrightness_exponent);
                distanceAttenuation = 1.0 /(targetDistance * targetDistance);
                illumination = inBeam * beamBrightness * beamProfile * distanceAttenuation;

                // Apply illumination correction to all channels
                for(int c = 0; c < 3; c++)
                {
                    output.at<cv::Vec3b>(y, x)[c] = cv::saturate_cast<uchar>(input.at<cv::Vec3b>(y, x)[c] *
                                                    illumination);
                }
            }
        }

        break;
    } // case 1

    case 2: // Uniform beam
    {
        // Loop over all pixels
        for(double y=0; y<input.rows-1; ++y)
        {
            for(double x=0; x<input.cols-1; ++x)
            {
                beamProfile = 1.0;
                inBeam = ((((x-beamCenterX) * (x-beamCenterX) / (beamWidth * beamWidth)) + ((y-beamCenterY) * (y-beamCenterY) / (beamHeight * beamHeight)))<= 1) ? 1.0 : 0.0;

                beamBrightness = beamBrightness_coeff * pow(10, beamBrightness_exponent);
                distanceAttenuation = 1.0 /(targetDistance * targetDistance);
                illumination = inBeam * beamBrightness * beamProfile * distanceAttenuation;

                // Apply illumination correction to all channels
                for(int c = 0; c < 3; c++)
                {
                    output.at<cv::Vec3b>(y, x)[c] = cv::saturate_cast<uchar>(input.at<cv::Vec3b>(y, x)[c] *
                                                    illumination);
                }
            }
        }
        break;
    } // case 2
    } // switch

} // Illuminate


/**
 * @brief
 *
 * @param img
 * @param rPerc
 * @param gPerc
 * @param bPerc
 */
void
adjustRGB(cv::Mat& img, double rPerc, double gPerc, double bPerc)
{
    dbg("")

    // Split imaage into RGB channels
    cv::Mat channels[3];
    cv::split(img, channels);

    // Adjust red channel
    dbg("")
    double rScale = double (rPerc / 100.0);
    channels[2].convertTo(channels[2], -1, rScale, 1.0); // Bottleneck

    // Adjust green channel
    dbg("")
    double gScale = double (gPerc / 100.0);
    channels[1].convertTo(channels[1], -1, gScale, 1.0); // Bottleneck

    // Adjust blue channel
    dbg("")
    double bScale = double (bPerc / 100.0);
    channels[0].convertTo(channels[0], -1, bScale, 1.0); // Bottleneck

    // Merge channels back into single image
    dbg("")
    cv::merge(channels, 3, img);
}


/**
 * @brief
 *
 * @param img
 * @param rPerc
 * @param gPerc
 * @param bPerc
 */
void
adjustRGB_2(cv::Mat& img, double rPerc, double gPerc, double bPerc)
{
    dbg("")

    double rScale = double (rPerc / 100.0);
    double gScale = double (gPerc / 100.0);
    double bScale = double (bPerc / 100.0);
    for(int i=0; i<img.rows-1; ++i)
    {
        for(int j=0; j<img.cols-1; ++j)
        {
            img.at<cv::Vec3b>(i,j)[0] = cv::saturate_cast<uchar>(bScale * img.at<cv::Vec3b>(i,j)[0]);
            img.at<cv::Vec3b>(i,j)[1] = cv::saturate_cast<uchar>(gScale * img.at<cv::Vec3b>(i,j)[1]);
            img.at<cv::Vec3b>(i,j)[2] = cv::saturate_cast<uchar>(rScale * img.at<cv::Vec3b>(i,j)[2]);
        }
    }
}


/**
 * @brief
 *
 * @param src
 * @param angle
 * @param dst
 */
void
rotateMat(cv::Mat& src, double angle, cv::Mat& dst)
{
    int len = std::max(src.cols, src.rows);
    cv::Point2f pt(len/2., len/2.);
    cv::Mat r = cv::getRotationMatrix2D(pt, angle, 1.0);
    cv::warpAffine(src, dst, r, cv::Size(len, len));
}


void
motionBlur(cv::Mat& img, int size, int direction)
{
    // Size must be at least 3 and odd
    if(size>2 && size%2==1)
    {
        cv::Mat kernel = cv::Mat::zeros(size, size, CV_64F);
        for(int i=0; i<size; ++i)
        {
            kernel.at<double>((size-1)/2, i) = 1;
        }

        kernel /= (double)size;

        rotateMat(kernel, (double)direction, kernel);

        cv::filter2D(img, img, -1, kernel); // Bottleneck
    }
}


/**
* @brief
*
* @param cMe
* @param fMe
* @param cWf
*/
void
computeCamToFixedTwist(cv::Mat& cMe, cv::Mat& fMe, cv::Mat cWf)
{
    cv::Mat eMf = cv::Mat::zeros(fMe.cols, fMe.rows, CV_64F);
    cv::invert(fMe, eMf, cv::DECOMP_SVD);
    cv::Mat cMf = cMe * eMf;
    HTransformToTwist(cMf, cWf);
}


/**
 * @brief
 *
 * @param J_w
 * @param J
 * @param ds
 * @param u
 * @param w
 */
void
computeDisturbance(cv::Mat& J_w, cv::Mat& J, cv::Mat& ds, cv::Mat& u, cv::Mat& w)
{
    cv::Mat J_w_inv(J_w.cols, J_w.rows, CV_64F);
    cv::invert(J_w, J_w_inv, cv::DECOMP_SVD);
    w = J_w_inv * (ds - J*u);
}


/**
 * @brief
 *
 * @param A
 * @param B
 * @param C
 */
void
MatStack(cv::Mat& A, cv::Mat& B, cv::Mat& C)
{
    double C_rows = A.rows + B.rows;
    double C_cols = A.cols + B.cols;
    C = cv::Mat(C_rows, C_cols, CV_64F);

    for(int i=0; i<A.rows; ++i)
    {
        C.push_back(A.row(i));
    }

    for(int i=0; i<B.rows; ++i)
    {
        C.push_back(B.row(i));
    }
}


/**
 * @brief
 *
 * @param cMe
 * @param fMe
 * @param fWb
 * @param J_q
 * @param bRw
 * @param ds
 * @param u_prev
 * @param disturbanceTerm
 */
void
computeDisturbanceTerm(cv::Mat& cMe, cv::Mat& fMe, cv::Mat& fWb, cv::Mat& J_q,\
                               cv::Mat& bWw, cv::Mat& ds, cv::Mat& u_prev, cv::Mat& disturbanceTerm)
{
    // Compute camera-to-fixed frame twist matrix
    cv::Mat cWf;
    computeCamToFixedTwist(cMe, fMe, cWf);

    // Compute vehicle Jacobian J_v.
    cv::Mat J_v = cWf * fWb;

    // Combine J_q and J_v into J
    cv::Mat J;
    MatStack(J_q, J_v, J);

    // Compute the disturbance Jacobian J_w
    cv::Mat J_w = J_v * bWw;

    // Compute disturbance w
    cv::Mat w;
    computeDisturbance(J_w, J, ds, u_prev, w);

    // Compute disturbance term
    cv::Mat J_inv;
    cv::invert(J, J_inv, cv::DECOMP_SVD);
    disturbanceTerm = -J_inv * J_w * w;
}


/**
 * @brief
 */
double
orientation(cv::Mat& I)
{
    cv::Mat G;
    cv::cvtColor(I, G, CV_BGR2GRAY);

    double M00 = 0.0;
    double M01 = 0.0;
    double M10 = 0.0;
    double M11 = 0.0;
    double M20 = 0.0;
    double M02 = 0.0;

    for(int x = 0; x < G.cols; ++x)
    {
        for(int y = 0; y < G.rows; ++y)
        {
            M00 += G.at<uchar>(y,x);
            M10 += G.at<uchar>(y,x) * x;
            M01 += G.at<uchar>(y,x) * y;
            M11 += G.at<uchar>(y,x) * x * y;
            M20 += G.at<uchar>(y,x) * x * x;
            M02 += G.at<uchar>(y,x) * y * y;
        }
    }

    double x_bar = M10 / M00;
    double y_bar = M01 / M00;

    double u11 = (M11/M00) - (x_bar * y_bar);
    double u20 = (M20/M00) - (x_bar * x_bar);
    double u02 = (M02/M00) - (y_bar * y_bar);


    dbg("M00 = " << M00)
    dbg("M01 = " << M01)
    dbg("M10 = " << M10)
    dbg("M11 = " << M11)
    dbg("M20 = " << M20)
    dbg("M02 = " << M02)

    dbg("u11 = " << u11)
    dbg("u02 = " << u02)
    dbg("u20 = " << u20)

    return 0.5 * atan2((2*u11), (u20-u02));

}


void
checkMatCompat(const cv::Mat& A, const cv::Mat& B)
{
    if(A.empty())
    {
        dbg("WARNING: A is empty!")
    }

    if(B.empty())
    {
        dbg("WARNING: B is empty!")
    }

    if(A.dims > 2)
    {
        dbg("WARNING: A.dims > 2")
    }

    if(B.dims > 2)
    {
        dbg("WARNING: B.dims > 2")
    }

    if(B.rows != A.rows)
    {
        dbg("WARNING: A.rows != B.rows")
    }

    if(B.cols != A.cols)
    {
        dbg("WARNING: A.cols != B.cols")
    }

    if(A.type() != B.type())
    {
        dbg("WARNING: A.type != B.type")
    }
}


/**
 * @brief
 *
 * @param A
 * @param B
 * @param C)
 */
void
hconcat(const cv::Mat& A, const cv::Mat& B, cv::Mat& C)
{
    checkMatCompat(A, B);

    if((A.rows == 0) || (A.cols == 0))
    {
        C = B.clone();
        return;
    }
    else if((B.rows == 0) || (B.cols==0))
    {
        C = A.clone();
        return;
    }

    C = cv::Mat(A.rows, A.cols + B.cols, CV_64F);
    for(int i=0; i<A.rows; ++i)
    {
        for(int j=0; j<A.cols; ++j)
        {
            C.at<double>(i, j) = A.at<double>(i, j);
        }
        for(int k=0; k<B.cols; ++k)
        {
            C.at<double>(i, A.cols + k) = B.at<double>(i, k);
        }
    }

}


/**
 * @brief
 *
 * @param A
 * @param B
 * @param C
 */
void
vconcat(const cv::Mat& A, const cv::Mat& B, cv::Mat& C)
{
    if((A.rows == 0) || (A.cols == 0))
    {
        C = B.clone();
        return;
    }
    else if((B.rows == 0) || (B.cols==0))
    {
        C = A.clone();
        return;
    }

    C = cv::Mat(A.rows+B.rows, A.cols, CV_64F);
    for(int i=0; i<A.rows; ++i)
    {
        for(int j=0; j<A.cols; ++j)
        {
            C.at<double>(i, j) = A.at<double>(i, j);
            C.at<double>(i+B.rows, j) = B.at<double>(i, j);
        }
    }
}


double
vectorL2Norm(std::vector<double>& v)
{
    double sum = 0.0;
    for(unsigned int i=0; i<v.size(); ++i)
    {
        sum += std::pow(v.at(i), 2);
    }
    return std::sqrt(sum);
}


double
vectorL2Norm(cv::Mat& v)
{
    double sum = 0.0;
    for(int i=0; i<v.rows; ++i)
    {
        sum += std::pow(v.at<double>(i), 2);
    }
    return std::sqrt(sum);
}


void
point3dReproject(cv::Mat& H, cv::Point3f& in, cv::Point3f& out)
{
    cv::Mat temp(4, 1, CV_64F);

    // 1. Transform the point to new camera frame given by transform H.
    for(unsigned int i=0; i<4; ++i)
    {
        temp.at<double>(i) = H.at<double>(i, 0) * in.x + H.at<double>(i, 1) * in.y +
                             H.at<double>(i, 2) * in.z + H.at<double>(i, 3) * 1.0;
    }

    double d = 1.0 / temp.at<double>(3);

    for(unsigned int i=0; i<4; ++i)
    {
        temp.at<double>(0) *= d;
    }

    // 2. Project new co-ordinates into the image plane.
    out.x = temp.at<double>(0) / temp.at<double>(2);
    out.y = temp.at<double>(1) / temp.at<double>(2);
    out.z = 1.0;
}


void
planeReproject(cv::Mat& H, double* in, double* out)
{
    // Transform between frames using cMo
    double A = H.at<double>(0, 0) * in[0] + H.at<double>(0, 1) * in[1] + H.at<double>(0, 2) * in[2];
    double B = H.at<double>(1, 0) * in[0] + H.at<double>(1, 1) * in[1] + H.at<double>(1, 2) * in[2];
    double C = H.at<double>(2, 0) * in[0] + H.at<double>(2, 1) * in[1] + H.at<double>(2, 2) * in[2];
    double D = in[3] - (H.at<double>(0, 3) * A + H.at<double>(1, 3) * B + H.at<double>(2, 3) * C);

    // Reproject
    out[0] = -A/D;
    out[1] = -B/D;
    out[2] = -C/D;
    out[3] = D;
}


void
L_moment_area(cv::Mat& L, double* parameters)
{
    double A = parameters[0];
    double B = parameters[1];
    double C = parameters[2];

    double X_g = parameters[3];
    double Y_g = parameters[4];
    double a = parameters[5];

    L = cv::Mat::zeros(1, 6, CV_64F);
    L.at<double>(0, 0) = -a * A;
    L.at<double>(0, 1) = -a * B;
    L.at<double>(0, 2) = 3*a*(A*X_g + B*Y_g) + 2*C*a;
    L.at<double>(0, 3) = 3*a*Y_g;
    L.at<double>(0, 4) = -3*a*X_g;
    L.at<double>(0, 5) = 0.0;
}


void
L_moment_CoM_x(cv::Mat& L, double* parameters)
{
    double A = parameters[0];
    double B = parameters[1];
    double C = parameters[2];

    double X_g = parameters[3];
    double Y_g = parameters[4];
    double epsilon = parameters[5];
    double nu_11 = parameters[6];
    double nu_20 = parameters[7];

    double inv_Z_g = A*X_g + B*Y_g + C;

    L = cv::Mat::zeros(1, 6, CV_64F);
    L.at<double>(0, 0) = -inv_Z_g;
    L.at<double>(0, 1) = 0.0;
    L.at<double>(0, 2) = X_g*inv_Z_g + A*epsilon*nu_20 + B*epsilon*nu_11;
    L.at<double>(0, 3) = X_g*Y_g*epsilon*nu_11;
    L.at<double>(0, 4) = X_g*Y_g*epsilon*nu_11;
    L.at<double>(0, 5) = Y_g;
}


void
L_moment_CoM_y(cv::Mat& L, double* parameters)
{
    double A = parameters[0];
    double B = parameters[1];
    double C = parameters[2];

    double X_g = parameters[3];
    double Y_g = parameters[4];
    double epsilon = parameters[5];
    double nu_11 = parameters[6];
    double nu_02 = parameters[7];

    double inv_Z_g = A*X_g + B*Y_g + C;

    L = cv::Mat::zeros(1, 6, CV_64F);
    L.at<double>(0, 0) = 0.0;
    L.at<double>(0, 1) = -inv_Z_g;
    L.at<double>(0, 2) = Y_g*inv_Z_g + A*epsilon*nu_11 + B*epsilon*nu_02;
    L.at<double>(0, 3) = 1 + Y_g*Y_g + epsilon*nu_02;
    L.at<double>(0, 4) = -X_g*Y_g*epsilon*nu_11;
    L.at<double>(0, 5) = -X_g;
}


void
L_moment_alpha(cv::Mat& L, double* parameters)
{
    double A = parameters[0];
    double B = parameters[1];
    double C = parameters[2];
    double Xg = parameters[3];
    double Yg = parameters[4];
    double epsilon = parameters[5];
    double mu_11 = parameters[6];
    double mu_20 = parameters[7];
    double mu_02 = parameters[8];
    double mu_12 = parameters[9];
    double mu_21 = parameters[10];
    double mu_30 = parameters[11];
    double mu_03 = parameters[12];
    double beta = parameters[13];
    double gamma = parameters[14];

    double d = (mu_20-mu_02)*(mu_20-mu_02) + 4*mu_11*mu_11;
    double DA = mu_20 + mu_02;
    double DA_2 = DA * DA;
    double mu11_2 = mu_11 * mu_11;

    double Lvx = (mu_11*DA*A)/d + (DA*mu_02 + (0.5)*d - (0.5)*DA_2) * (B/d);
    double Lvy = (DA*mu_02 - (0.5)*d -(0.5)*DA_2)*(A/d) - (B*mu_11*DA)/d;
    double Lwx = (beta*(mu_12*(mu_20-mu_02) + mu_11*(mu_03-mu_21)) + gamma*Xg*(mu_02*(mu_20-mu_02) - 2*mu11_2) + gamma*Yg*mu_11*(mu_20+mu_02))/d;
    double Lwy = (beta*(mu_21*(mu_02-mu_20) + mu_11*(mu_30-mu_12)) + gamma*Xg*mu_11*(mu_20+mu_02) + gamma*Yg*(mu_20*(mu_02-mu_20)-2*mu11_2))/d;
    double Lvz = B*Lwx - A*Lwy;

    L = cv::Mat::zeros(1, 6, CV_64F);
    L.at<double>(0, 0) = Lvx;
    L.at<double>(0, 1) = Lvy;
    L.at<double>(0, 2) = Lvz;
    L.at<double>(0, 3) = Lwx;
    L.at<double>(0, 4) = Lwy;
    L.at<double>(0, 5) = -1;
}


void
getMomentFeatures(cv::Mat& inputImage, cv::Mat& outputFeatures)
{
     outputFeatures = cv::Mat::zeros(6, 1, CV_64F);
     cv::Moments m = cv::moments(inputImage, true);

     double x_g = m.m10 / m.m00;
     double y_g = m.m01 / m.m00;
     double area = m.m00;
     double theta = 0.5 * atan2((2.0*m.mu11), (m.mu20-m.mu02));

     double Del = (m.mu20 - m.mu02)*(m.mu20 - m.mu02) + 4*m.mu11*m.mu11;
     //double alpha = (m.mu11*(m.mu20 + m.mu02)) / Del;

     double s_1 = m.mu03 - 3*m.mu21;
     double s_2 = m.mu30 - 3*m.mu12;
     double P_2 = Del / ((m.mu20 + m.mu02)*(m.mu20 + m.mu02));
     double P_3 = (area * (s_1*s_1 + s_2*s_2)) / ((m.mu20 + m.mu02)*(m.mu20 + m.mu02)*(m.mu20 + m.mu02));

     outputFeatures = cv::Mat(6, 1, CV_64F);
     outputFeatures.at<double>(0) = x_g;
     outputFeatures.at<double>(1) = y_g;
     outputFeatures.at<double>(2) = area;
     outputFeatures.at<double>(3) = P_2;
     outputFeatures.at<double>(4) = P_3;
     outputFeatures.at<double>(5) = theta;
}


void
getMomentFeatures(std::vector<cv::Point>& inputPoints, cv::Mat& outputFeatures)
{
     outputFeatures = cv::Mat::zeros(6, 1, CV_64F);
     cv::Moments m = cv::moments(inputPoints, true);

     double x_g = m.m10 / m.m00;
     double y_g = m.m01 / m.m00;
     double area = m.m00;
     double theta = 0.5 * atan2((2*m.mu11), (m.mu20-m.mu02));

     double Del = (m.mu20 - m.mu02)*(m.mu20 - m.mu02) + 4*m.mu11*m.mu11;
     //double alpha = (m.mu11*(m.mu20 + m.mu02)) / Del;

     double s_1 = m.mu03 - 3*m.mu21;
     double s_2 = m.mu30 - 3*m.mu12;
     double P_2 = Del / ((m.mu20 + m.mu02)*(m.mu20 + m.mu02));
     double P_3 = (area * (s_1*s_1 + s_2*s_2)) / ((m.mu20 + m.mu02)*(m.mu20 + m.mu02)*(m.mu20 + m.mu02));

     outputFeatures = cv::Mat(6, 1, CV_64F);
     outputFeatures.at<double>(0) = x_g;
     outputFeatures.at<double>(1) = y_g;
     outputFeatures.at<double>(2) = area;
     outputFeatures.at<double>(3) = P_2;
     outputFeatures.at<double>(4) = P_3;
     outputFeatures.at<double>(5) = theta;
}

double getSpecificMoment(cv::Mat& inputImage, int i, int j)
{
    double moment = 0.0;

    #pragma omp parallel
    {
        #pragma omp for nowait
        for(int n=0; n<inputImage.rows; ++n)
        {
            for(int m=0; m<inputImage.cols; ++m)
            {
                if(inputImage.at<double>(n, m) == 0)
                {
                    moment += std::pow(n, i) * std::pow(m, j);
                }
            }
        }
    }

    return moment;
}


void
getMomentFeatures2(cv::Mat& inputImage, cv::Mat& outputFeatures)
{
     outputFeatures = cv::Mat::zeros(6, 1, CV_64F);

     double m00, m01, m10, m11, m20, m02, m21, m12, m30, m03 = 0.0;

     #pragma omp parallel
     {
         #pragma omp for nowait
         for(int n=0; n<inputImage.rows; ++n)
         {
             for(int m=0; m<inputImage.cols; ++m)
             {
                 if(inputImage.at<double>(n, m) == 0)
                 {
                     double x = n;
                     double y = m;

                     m00 += 1.0;

                     m10 += x;
                     m01 += y;
                     m11 += x * y;

                     double x2 = x * n;
                     double y2 = y * m;

                     m20 += x2;
                     m02 += y2;

                     m12 += x * y2;
                     m21 += x2 * y;

                     double x3 = x2 * n;
                     double y3 = y2 * m;

                     m30 += x3;
                     m03 += y3;
                 }
             }
         }
     }

     double x_g = m10 / m00;
     double y_g = m01 / m00;

     double mu11 = m11 - x_g * m01;
     double mu20 = m20 - x_g * m10;
     double mu02 = m02 - y_g * m01;
     double mu21 = m21 - 2 * x_g * m11 - y_g * m20 + 2 * x_g * x_g * m01;
     double mu12 = m12 - 2 * y_g * m11 - x_g * m02 + 2 * y_g * y_g * m10;
     double mu30 = m30 - 3 * x_g * m20 + 2 * x_g * x_g * m10;
     double mu03 = m03 - 3 * y_g * m02 + 2 * y_g * y_g * m01;

     double area = m00;
     double theta = 0.5 * atan2((2.0*mu11), (mu20-mu02));

     double Del = (mu20 - mu02)*(mu20 - mu02) + 4*mu11*mu11;
     //double alpha = (mu11*(mu20 + mu02)) / Del;

     double s_1 = mu03 - 3*mu21;
     double s_2 = mu30 - 3*mu12;
     double P_2 = Del / ((mu20 + mu02)*(mu20 + mu02));
     double P_3 = (area * (s_1*s_1 + s_2*s_2)) / ((mu20 + mu02)*(mu20 + mu02)*(mu20 + mu02));

     outputFeatures = cv::Mat(6, 1, CV_64F);
     outputFeatures.at<double>(0) = x_g;
     outputFeatures.at<double>(1) = y_g;
     outputFeatures.at<double>(2) = area;
     outputFeatures.at<double>(3) = P_2;
     outputFeatures.at<double>(4) = P_3;
     outputFeatures.at<double>(5) = theta;
}

} // utils namespace
} // vs namespace
