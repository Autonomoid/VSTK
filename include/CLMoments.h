#ifndef CLMOMENTS_H
#define CLMOMENTS_H

#include "../include/utils.hpp"

class CLMoments
{
public:
    CLMoments();
    template<class T> inline void setImage(T& inputImage, unsigned int order);
    void setPlane(double A_, double B_, double C_);
    cv::Mat& getInteractionMatrix();

private:
    void computeMoments(cv::Mat& inputImage, unsigned int order);

    void build_L_CoM_x();
    void build_L_CoM_y();
    void build_L_area();
    void build_L_C_x();
    void build_L_C_y();
    void build_L_alpha();

    double getMu(int i, int J);
    cv::Mat getLMu(int i, int j);
    double muList[4][4];
    double inv[10]; // invariants
    double CInv[10]; // C invariants

    cv::Mat getLInv_1();
    cv::Mat getLInv_2();
    cv::Mat getLCInv_1();

    // Interaction matrix
    cv::Mat L;

    // Plane parameters
    double A;
    double B;
    double C;

    // Raw moments
    cv::Moments m;
    std::vector<double> values;

    double area;
    double X_g;
    double Y_g;
    double inv_Z_g;
    double epsilon1;
    double epsilon2;
};

// Input type is std::vector<cv::Point> or cv::Mat
template <class T>
void
CLMoments::setImage(T& input, unsigned int order)
{
    this->m = cv::moments(input, true);

    this->area = m.m00;
    this->X_g = (m.m10 / m.m00);
    this->Y_g = (m.m01 / m.m00);

    this->epsilon1 = (4/m.m00)*(A*m.mu20 + B*m.mu11);
    this->epsilon2 = (4/m.m00)*(A*m.mu11 + B*m.mu02);

    this->muList[2][0] =  m.mu20;
    this->muList[1][1] =  m.mu11;
    this->muList[0][2] =  m.mu02;
    this->muList[3][0] =  m.mu30;
    this->muList[2][1] =  m.mu21;
    this->muList[1][2] =  m.mu12;
    this->muList[0][3] =  m.mu03;

    // Compute C invariant 1
    this->inv[0] = -m.mu20*m.mu02 + m.mu11*m.mu11;
    this->inv[1] = (m.mu20-m.mu02)*(m.mu20-m.mu02) + 4*m.mu11*m.mu11;
    this->CInv[0] = this->inv[0] / this->inv[1];

    std::cout << "mu30 = " << m.mu30 << std::endl;
    std::cout << "mu21 = " << m.mu21 << std::endl;
    std::cout << "mu12 = " << m.mu12 << std::endl;
}

#endif // CLMOMENTS_H
