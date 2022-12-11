#include "../include/CLMoments.h"

CLMoments::CLMoments()
{}


void
CLMoments::computeMoments(cv::Mat& inputImage, unsigned int order)
{
    std::vector<double> temp(order*order, 0.0);
    this->values.assign(order*order, 0.0);

    // Loop over every pixel in the binary image.
    for(unsigned int i=0; i<inputImage.cols; ++i)
    {
        for(unsigned int j=0; j<inputImage.rows; ++j)
        {
            // If the pixel equals 1:
            if(inputImage.at<double>(j, i) == 1)
            {
                // Convert the pixel co-ordinates to metric co-ordinates.
                cv::Point2f p(i, j);

                double u0, v0, fx, fy, px, py = 0.0; // temp line
                cv::Point2f m = vs::utils::pixelToMeter(p, u0, v0, fx, fy, px, py);

                temp[0]=1;

                // Compute the x moments
                for(unsigned int i=1; i<order; ++i)
                {
                    temp[i] = temp[i-1] * m.x;
                }

                // Compute the y moments
                for(unsigned int j=order; j<order*order; j+=order)
                {
                    temp[j] = temp[j-order] * m.y;
                }

                // Compute the xy moments
                for(unsigned int j=1; j<order; ++j)
                {
                    for(unsigned int i=1; i<order-j; ++i)
                    {
                        temp[j*order+i] = temp[j*order]*temp[i];
                    }
                }

                // Accumulate the moment values
                for(unsigned int k=0; k<order; ++k)
                {
                    for(unsigned int l=0; l<order-k; ++l)
                    {
                        values[k*order+l] += temp[k*order+l];
                    }
                }

            }// if
        }// for j
    }// for i
} // computeMoments

void
CLMoments::setPlane(double A_, double B_, double C_)
{
    this->A = A_;
    this->B = B_;
    this->C = C_;
}


cv::Mat&
CLMoments::getInteractionMatrix()
{
    this->inv_Z_g = this->A*this->X_g + this->B*this->Y_g + this->C; // = 1/Z_g
    this->L = cv::Mat::zeros(6, 6, CV_64F);
    this->build_L_CoM_x();
    this->build_L_CoM_y();
    this->build_L_area();
    this->build_L_C_x();
    this->build_L_C_y();
    this->build_L_alpha();
    return L;
}


void
CLMoments::build_L_CoM_x()
{
    // From Chaumette "A First Step Towards Visual Servoing Using Image Moments"
    L.at<double>(0, 0) = -inv_Z_g;
    L.at<double>(0, 1) = 0.0;
    L.at<double>(0, 2) = X_g*inv_Z_g + epsilon1;
    L.at<double>(0, 3) = (X_g*Y_g) + (4*m.mu11/area);
    L.at<double>(0, 4) = -(1 + (X_g*X_g) + (4*m.mu20/area));
    L.at<double>(0, 5) = Y_g;
}


void
CLMoments::build_L_CoM_y()
{
    // From Chaumette "A First Step Towards Visual Servoing Using Image Moments"
    L.at<double>(1, 0) = 0.0;
    L.at<double>(1, 1) = -inv_Z_g;
    L.at<double>(1, 2) = Y_g*inv_Z_g + epsilon2;
    L.at<double>(1, 3) = 1 + (Y_g*Y_g) + (4*m.mu02/area);
    L.at<double>(1, 4) = -(X_g*Y_g) - (4*m.mu11/area);
    L.at<double>(1, 5) = -X_g;
}


void
CLMoments::build_L_area()
{
    // From Chaumette "A First Step Towards Visual Servoing Using Image Moments"
    L.at<double>(2, 0) = -area * A;
    L.at<double>(2, 1) = -area * B;
    L.at<double>(2, 2) = area*(3*inv_Z_g - C);
    L.at<double>(2, 3) = 3*area*Y_g;
    L.at<double>(2, 4) = -3*area*X_g;
    L.at<double>(2, 5) = 0.0;
}


void
CLMoments::build_L_C_x()
{
    // From Tahri & Chaumette "Point-based and region-based image moments for visual servoing of planar object"
    //L.at<double>(3, 3) = getLCInv_1().at<double>(1,3); //Ciwx;
    //L.at<double>(3, 4) = getLCInv_1().at<double>(1,4); //Ciwy;
}


void
CLMoments::build_L_C_y()
{
    // From Tahri & Chaumette "Point-based and region-based image moments for visual servoing of planar object"
    //L.at<double>(4, 3) = Cjwx;
    //L.at<double>(4, 4) = Cjwy;
}


void
CLMoments::build_L_alpha()
{
    // From Chaumette "A First Step Towards Visual Servoing Using Image Moments"

    double DA = m.mu20 + m.mu02;
    double DA_2 = DA * DA;
    double mu11_2 = m.mu11 * m.mu11;

    double Delta = (m.mu20-m.mu02)*(m.mu20-m.mu02) + (4*m.mu11*m.mu11);
    double alpha = (m.mu11*DA)/Delta;
    double beta = (2*mu11_2 + m.mu02*(m.mu02 - m.mu20)) / Delta;
    double gamma = (2*mu11_2 + m.mu20*(m.mu20 - m.mu02)) / Delta;
    double delta = 5*(m.mu12*(m.mu20 - m.mu02) + m.mu11*(m.mu03 - m.mu21)) / Delta;
    double nu = 5*(m.mu21*(m.mu02 - m.mu20) + m.mu11*(m.mu30 - m.mu12)) / Delta;

    double Lvx = alpha*A + beta*B;
    double Lvy = -gamma*A - alpha*B;
    double Lwx = -beta*X_g + alpha*Y_g + delta;
    double Lwy = alpha*X_g - gamma*Y_g + nu;
    double Lvz = B*Lwx - A*Lwy;

    L.at<double>(5, 0) = Lvx;
    L.at<double>(5, 1) = Lvy;
    L.at<double>(5, 2) = Lvz;
    L.at<double>(5, 3) = Lwx;
    L.at<double>(5, 4) = Lwy;
    L.at<double>(5, 5) = -1;

    std::cout << "Delta = " << Delta << std::endl;

}


double
CLMoments::getMu(int i, int j)
{
    return this->muList[i][j];
}


// Get the interaction matrix for central moment mu_ij
cv::Mat
CLMoments::getLMu(int i, int j)
{
    cv::Mat LMu(1, 6, CV_64F);
    double del = 1.0;
    double epsilon = 4;
    LMu.at<double>(0, 0) = -(i+del)*A*getMu(i,j) - i*B*getMu(i-1,j+1);
    LMu.at<double>(0, 1) = -j*A*getMu(i+1,j-1) - (j+del)*B*getMu(i,j);
    LMu.at<double>(0, 3) = (i+j+3*del)*getMu(i,j+1) + (i+j+3*del)*Y_g*getMu(i,j)
                           + i*X_g*getMu(i-1,j+1) - i*epsilon*m.nu11*getMu(i-1,j) - j*epsilon*m.nu02*getMu(i,j-1);
    LMu.at<double>(0, 4) = -(i+j+3*del)*getMu(i+1,j) - (2*i+j+3*del)*X_g*getMu(i,j)
                           - j*Y_g*getMu(i+1,j-1) + i*epsilon*m.nu20*getMu(i-1,j) + j*epsilon*m.nu11*getMu(i,j-1);
    LMu.at<double>(0, 2) = -A*LMu.at<double>(0, 4) + B*LMu.at<double>(0, 3) + (i+j+2*del)*C*getMu(i,j);
    LMu.at<double>(0, 5) = i*getMu(i-1,j+1) - j*getMu(i+1,j-1);

    std::cout << "\nLMu(" << i << "," << j << ") = " << std::endl << LMu << std::endl;
    return LMu.clone();
}


// Get the interaction matrix for moment inveriant I_1
cv::Mat
CLMoments::getLInv_1()
{
    cv::Mat LInv_1(1, 6, CV_64F);
    LInv_1 = -(getLMu(2,0)*m.mu02) - (m.mu20*getLMu(0,2)) + 2*m.mu11*getLMu(1,1);

    std::cout << "\nInv_1 = " << std::endl << LInv_1 << std::endl;

    return LInv_1.clone();
}


// Get the interaction matrix for moment inveriant I_2
cv::Mat
CLMoments::getLInv_2()
{
    cv::Mat LInv_2(1, 6, CV_64F);
    LInv_2 = (-2*m.mu20+2*m.mu02)*getLMu(0,2) + 8*m.mu11*getLMu(1,1) + (2*m.mu20-2*m.mu02)*getLMu(2,0);

    std::cout << "\nLInv_2 = " << std::endl << LInv_2 << std::endl;

    return LInv_2.clone();
}


// Get the interaction matrix for C invariant 1, = I_1 / I_2
cv::Mat
CLMoments::getLCInv_1()
{
    cv::Mat LInv_1(1, 6, CV_64F);
    LInv_1 = (this->inv[1]*this->getLInv_1() - this->inv[0]*this->getLInv_2()) / (this->inv[1]*this->inv[1]);

    std::cout << "\nLCInv_1 = " << std::endl << LInv_1 << std::endl;

    return LInv_1.clone();
}
