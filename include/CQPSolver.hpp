/////////////////////////////////////////////////////////////////
//
// Name: CQPSolver.h
//
// Active-set Quadratic Program Solver based on the ViSP HQP
// solver by Olivier Kermorgant.
//
// At each iteration solve:
// min || Ak.x - Bk || + ||w||
// st. AA.x = BB
// CC.x < DD
// Ck.x - w < Dk
//
// This problem is written canonically as:
//
// min  1/2 X^t.Q.X^t - r^t.X
//
// s.t. A.X = b and C.X < d
//
// with
//
// X = (x,w)
// Q = Akt.Ak
// r = Akt.B
//

#ifndef CQPSOLVER_H
#define CQPSOLVER_H

#include <vector>
#include <iostream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace vs
{

/**
 * @brief 
 */
class CQPSolver
{
  public:
    CQPSolver(const cv::Mat& _Q, const cv::Mat& _r, const cv::Mat& _A, const cv::Mat _b, const cv::Mat& _C, const cv::Mat& _d);
    int solve();
    cv::Mat getSolution() const;
    void setEpsilon(double);

  private:
    void init();
    int checkMatrixDims();
    int checkTrivialSoln();
    void buildMatrices();
    void computeRegularizedSoln();
    void findMostViolatedConstraint();
    void checkSolution();
    void findLeastViolatedActiveConstraint();
    void checkPrevActiveSets();

    // Input Matrices
    cv::Mat Q; // 4x4
    cv::Mat r; // 4x1
    cv::Mat A; // 0x4
    cv::Mat b; // 0x0
    cv::Mat C; // 6x4
    cv::Mat d; // 6x1
    cv::Mat soln; // 4x1
    cv::Mat tempSoln; // 4x1

    // Intermediate matrices
    cv::Mat KKT;
    cv::Mat v;
    cv::Mat X;

    // Matrix dimensions
    uint Q_cols;
    uint Q_rows;
    uint A_cols;
    uint A_rows;
    uint C_cols;
    uint C_rows;
    uint r_cols;
    uint r_rows;
    uint b_cols;
    uint b_rows;
    uint d_cols;
    uint d_rows;

    // Active set:
    int nAct;
    std::vector<bool> activeSet;
    std::vector<std::vector<bool> > prevActiveSets;

    // Inequalities
    int ineqIndex;
    uint ineqCount;
    double maxViolation;
    double minLagrangeMultiplier;
    double zeroThresh;

    // Regularization
    double lambda;

    // Solution error
    double errorBest;
    double errorCurrent;

    double epsilon;
};

} // namespace

#endif

