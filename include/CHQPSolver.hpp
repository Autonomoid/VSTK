/*
 * Hierarchical Quadratic Program Solver
 * Based on the CHQPSolver solver by Olvier Kermergant
 */

//#define DEBUG
#ifdef DEBUG
    #define dbg(msg) std::cout << "[DBG] " << __TIME__ << ":" << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": " << msg << std::endl;
#else
    #define dbg(msg)
#endif

#include <vector>
#include <iostream>
#include <string>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "CQPSolver.hpp"
#include "utils.hpp"

namespace vs
{

typedef enum {
    EQUALITY,
    INEQUALITY
} qpType;

/**
 * @brief 
 */
class CHQPSolver
{
    public:
        CHQPSolver();
        void updateXdim(const uint& colM);
        void addEquality(cv::Mat& A, cv::Mat& b, const unsigned int& h);
        void addInequality(cv::Mat& C, cv::Mat& d, const unsigned int& h);
        int solveCascade(cv::Mat& solution);
        void setEpsilon(double);

    private:
        void initKMatrices();
        void updateIndex(const qpType& type, uint& indM, uint & indV, uint& indS, uint& indSd);

        bool initStruct;
        int hMax;
        double rho;
        int nX;
        double thresh0;
        uint indE;
        uint indM;
        uint indV;
        uint indS;
        uint indSd;
        uint indAct;
        std::vector<uint> indA;
        std::vector<uint> indC;
        
        cv::Mat LJ;
        cv::Mat e;
        cv::Mat vPos;
        cv::Mat vNeg;
        cv::Mat xPrev;

        // Inputs
        std::vector<cv::Mat*> Ms;
        std::vector<cv::Mat*> Vs;
        std::vector<qpType> Ts;
        std::vector<uint> Hs;
        std::vector<std::string> Labels;

        // Constraints for individual QP
        std::vector<cv::Mat> Ak;
        std::vector<cv::Mat> Bk;
        std::vector<cv::Mat> Ck;
        std::vector<cv::Mat> Dk;

        // Stacked constraints
        cv::Mat AA;
        cv::Mat CC;
        cv::Mat BB;
        cv::Mat DD;

        // Regularization term
        double epsilon;

};

} // namespace
