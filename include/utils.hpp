#ifndef CUTILS_H
#define CUTILS_H

#include <iostream>

#include <boost/interprocess/managed_shared_memory.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace vs
{
namespace utils
{

extern const char* MEMORY_NAME;

// Shared image header
typedef struct{
  cv::Size size;
  int type;
  int frameID;
  boost::interprocess::managed_shared_memory::handle_t handle;
} SharedImageHeader;

    cv::Point2f meterToPixel(cv::Point2f p, double u0, double v0, double fx, double fy, double px, double py);

    cv::Point2f pixelToMeter(cv::Point2f p, double u0, double v0, double fx, double fy, double px, double py);

    cv::Point2f addRadialDistortion(cv::Point2f p, double k);

    double radToDeg(double rad);

    double degToRad(double deg);

    void worldToImage(std::vector<cv::Point2f>& worldPoints, std::vector<cv::Point2f>& imagePoints, \
                      double fx, double fy, double px, double py, double u0, double v0, double k);

    std::vector<double> rotationMatToVec(cv::Mat& R);

    void VecToRotationMat(std::vector<double>& v, cv::Mat& R);

    void rotationMatToAxisAngle(cv::Mat& R, double theta, cv::Mat& u);

    double vectorL2Norm(std::vector<double>& v);

    double vectorL2Norm(cv::Mat& v);

    void PoseToRotationMatrix(cv::Mat& pose, cv::Mat& M);

    void PoseToHTransform(cv::Mat& pose, cv::Mat& H);

    void HTransformToPose(const cv::Mat& H, cv::Mat& pose);

    void HTransformToRotationMatrix(const cv::Mat& H, cv::Mat& R);

    void combineHTransforms(const cv::Mat& A, const cv::Mat& B, cv::Mat& C);

    void ThreeVectorToSkewMatrix(std::vector<double>& v, cv::Mat& M);
  
    void HTransformToTwist(cv::Mat& H, cv::Mat& twist);
 
    void PoseToTwist(cv::Mat& pose, cv::Mat& twist);

    std::vector<double> vectorDiff(std::vector<double>& a, std::vector<double>& b);
  
    double distributeControl(double approachInput, double interventionInput, double criterion, \
               double approachThreshold, double interventionThreshold);

    double FourierSineSynth(double time, std::vector<std::pair<double,double> >& input);

    void MatStack(cv::Mat& A, cv::Mat& B, cv::Mat& C);

    void saltAndPepperNoise(cv::Mat& img, int percentage);

    void gaussianNoise(cv::Mat& img, int deviation);

    void illuminate(const cv::Mat& input, cv::Mat& output, int mode, double theta_x,\
                    double theta_y, double targetDistance, int beamBrightness_coeff,\
                    int beamBrightness_exponent, int beamCenterX, int beamCenterY,\
                    int beamHeight, int beamWidth);

    void adjustRGB(cv::Mat& img, double rPerc, double gPerc, double bPerc);

    void adjustRGB_2(cv::Mat& img, double rPerc, double gPerc, double bPerc);

    void rotateMat(cv::Mat& src, double angle, cv::Mat& dst);

    void motionBlur(cv::Mat& img, int size, int direction);

    void computeCamToFixedTwist(cv::Mat& cMe, cv::Mat& fMe, cv::Mat cWf);

    void computeDisturbance(cv::Mat& J_w, cv::Mat& J, cv::Mat& ds, cv::Mat& u, cv::Mat& w);

    void computeDisturbanceTerm(cv::Mat& cMe, cv::Mat& fMe, cv::Mat& fWb, cv::Mat& J_q,\
                                cv::Mat& bWw, cv::Mat& ds, cv::Mat& u_prev, cv::Mat& disturbanceTerm);

    double orientation(cv::Mat& I);

    void checkMatCompat(const cv::Mat& A, const cv::Mat& B);

    void hconcat(const cv::Mat& A, const cv::Mat& B, cv::Mat& C);

    void vconcat(const cv::Mat& A, const cv::Mat& B, cv::Mat& C);

    void point3dReproject(cv::Mat& H, cv::Point3f& in, cv::Point3f& out);

    void planeReproject(cv::Mat& H, double* in, double* out);

    void L_moment_CoM_x(cv::Mat& L, double* parameters);

    void L_moment_CoM_y(cv::Mat& L, double* parameters);

    void L_moment_alpha(cv::Mat& L, double* parameters);

    void L_moment_area(cv::Mat& L, double* parameters);

    void getMomentFeatures(cv::Mat& inputImage, cv::Mat& outputFeatures);
    void getMomentFeatures(std::vector<cv::Point>& inputPoints, cv::Mat& outputFeatures);
    double getSpecificMoment(cv::Mat& inputImage, int i, int j);
    void getMomentFeatures2(cv::Mat& inputImage, cv::Mat& outputFeatures);

  /**
   * @brief Low pass filter functor 
   */
  class LowPassFilter
  {
    public:
      LowPassFilter(double p, double s) : state(p), sensitivity(s)
      {
        // 0.0 <= sensitivity <= 1.0
        if(sensitivity > 1.0) sensitivity = 1.0;
        if(sensitivity < 0.0) sensitivity = 0.0;
      }

      // Overload the () operator
      double operator()(const double input)
      {
        this->state = (1-this->sensitivity)*this->state + this->sensitivity*input;
        return state;
      }

    private:
      double state;
      double sensitivity;
  };

} // namespace utils
} // namespace vs


#endif
