/*
 * Hierarchical Quadratic Program Solver
 * Based on the HQP solver by Olvier Kermergant
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

#include "qpsolver.h"
#include "utils.h"

typedef enum {
    EQUALITY,
    INEQUALITY,
} qpType;

/**
 * @brief 
 */
class hqp
{
    public:
        hqp();
        void addEquality(cv::Mat& A, cv::Mat& b, const unsigned int& h);
        //void addEqualityConstraint(vs::equalityConstraint&);

        void addInequality(cv::Mat& C, cv::Mat& d, const unsigned int& h);
        //void addInEqualityConstraint(vs::inequalityConstraint&);

        int solveCascade(cv::Mat& solution);

    private:
        void initKMatrices();
        void updateIndex(const qpType& type, uint& indM, uint & indV, uint& indS, uint& indSd);

        bool initStruct;
        int hMax;
        double lambda;
        double rho;
        int nX;
        double thresh0;
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

        std::vector<cv::Mat*> Ms;
        std::vector<cv::Mat*> Vs;
        std::vector<qpType> Ts;
        std::vector<uint> Hs;
        std::vector<std::string> Labels;
        std::vector<cv::Mat> Ak;
        std::vector<cv::Mat> Bk;
        std::vector<cv::Mat> Ck;
        std::vector<cv::Mat> Dk;
};


/**
 * @brief 
 */
hqp::hqp()
: initStruct(false), hMax(0), lambda(1), rho(0.05), nX(0),
  thresh0(1e-6), indM(0), indV(0), indS(0), indSd(0), indAct(0),    
  indA(std::vector<uint>(hMax+1, 0)),
  indC(std::vector<uint>(hMax+1, 0))
{
    dbg("")
}

/**
 * @brief kkk
 *
 * @param C
 * @param d
 * @param h
 * @param label
 */
void 
hqp::addInequality(cv::Mat& C, cv::Mat& d, const uint& h)
{
    dbg("")

    Ms.push_back(&C);
    Vs.push_back(&d);
    Ts.push_back(INEQUALITY);
    Hs.push_back(h);
    if (uint(hMax) < h)
        hMax = h;
}


void 
hqp::addEquality(cv::Mat& A, cv::Mat& b, const uint& h)
{
    dbg("")

    Ms.push_back(&A);
    Vs.push_back(&b);
    Ts.push_back(EQUALITY);
    Hs.push_back(h);
    if (uint(hMax) < h)
        hMax = h;
}


/**
 * @brief 
 *
 * @param solution
 */
void 
hqp::solveCascade(cv::Mat& solution)
{
    dbg("")

    dbg("Setup Ak, Bk, Ck, Dk")
    initKMatrices();
       
    dbg("init meta-matrices AA, BB, CC, DD")
    cv::Mat AA(0, nX, CV_64F);
    cv::Mat CC(0, nX, CV_64F);
    cv::Mat BB;
    cv::Mat DD;

    dbg("init canonical QP matrices")
    cv::Mat Q, A, C, Iw;
    cv::Mat r, B, d, X, x;
    uint nW = 0;

    cv::Mat tempA;
    cv::Mat tempB;

    for(int h=0; h<hMax+1; ++h)
    {
        nW = Ck[h].rows;
        dbg("nW = Ck[" << h << "].rows = " << nW)

        if(nW != 0 || Bk[h].rows != 0)		        {

            dbg("build canonical QP matrices Q, r, A, B, C, d")
            Iw = cv::Mat::eye(nW, nW, CV_64F);

            dbg("Q = [Ak[h], 0; 0, Iw]")
            dbg("Ak[" << h << "] = " << std::endl << Ak[h] << std::endl)
                
            //tempA = cv::Mat(Ak[h].rows, Ak[h].cols+nW, CV_64F);
            //tempB = cv::Mat(Ak[h].rows, Ak[h].cols+nW, CV_64F);
            //cv::hconcat(Ak[h], cv::Mat::zeros(Ak[h].rows, nW, CV_64F), tempA);
            //cv::hconcat(cv::Mat::zeros(nW, Ak[h].cols, CV_64F), Iw, tempB);
            utils::hconcat(Ak[h], cv::Mat::zeros(Ak[h].rows, nW, CV_64F), tempA);
            utils::hconcat(cv::Mat::zeros(nW, Ak[h].cols, CV_64F), Iw, tempB);
            utils::vconcat(tempA, tempB, Q);

            dbg("r = [Bk[h]; 0]")
            utils::vconcat(Bk[h], cv::Mat::zeros(nW, 1, CV_64F), r);

            r = Q.t()*r;
            Q = Q.t()*Q;

            if(AA.rows > 0)
            {
                dbg("A = [AA, 0; 0, 0]")
                utils::hconcat(AA, cv::Mat::zeros(AA.rows, nW, CV_64F), tempA);
                tempB = cv::Mat::zeros(nW, 2*AA.cols, CV_64F);
                utils::vconcat(tempA, tempB, A);

                dbg("b = [BB; 0]")
                utils::vconcat(BB, cv::Mat::zeros(nW, 1, CV_64F), B);
            }
            else
            {
                A = cv::Mat(0, nX+nW, CV_64F);
                B.resize(0);
            }

            if(CC.rows + nW > 0)
            {
                dbg("C = [CC, 0; Ck[h], -Iw]")
                dbg("CC = " << std::endl << CC << std::endl)
                utils::hconcat(CC, cv::Mat::zeros(CC.rows, nW, CV_64F), tempA);
                utils::hconcat(Ck[h], -Iw, tempB);
dbg("")
                utils::vconcat(tempA, tempB, C);

                dbg("d = [DD; Dk[h]]")
                utils::vconcat(DD, Dk[h], d);
            }
            else
            {
                C = cv::Mat(0, nX+nW, CV_64F);
                d.resize(0);
            }

            dbg("solve program")
            qpsolver solver(Q, r, A, B, C, d);
            solver.solve();

            dbg("extract solution")
            X = solver.getSolution();

            dbg("X = " << std::endl << X << std::endl)
            x = X(cv::Rect(0, 0, 1, nX));

            dbg("unless we are at the last iteration, update AA, BB, CC, DD and loop")
            if(h != hMax)
            {
                dbg("equalities stay equalities" << std::endl << "AA = [AA; Ak[h]]")
                AA.push_back(Ak[h]);
                
                dbg("BB = [BB; Ak[h]*x]")
                BB.push_back(Ak[h]*x);

                for(uint i=0; i<nW; ++i)
                {
                    cv::Mat temp = Ck[h].row(i+1)*x;
                    double t = temp.at<double>(0);
                    if(t < Dk[h].at<double>(i) + thresh0)
                    {
                        dbg("ensured inequalities stay inequalities" << std::endl <<
                        "CC = [CC; Ck[h].row[i+1]]")
                        CC.push_back(Ck[h].row(i+1));
                        
                        dbg("DD = [DD; Dk[h][i]]")
                        DD.push_back(Dk[h].at<double>(i));
                    }
                    else
                    {	// violated inequalities become equalities
                        AA.push_back(Ck[h].row(i+1));
                        BB.push_back(Ck[h].row(i+1)*x);
                    }
                }
            }
        }
    }

    solution = xPrev = x;
}


/**
 * @brief 
 */
void
hqp::initKMatrices()
{
    dbg("")

    // initialize the matrices Ak, Bk, Ck, Dk from stored values
    // if it is the first time, this is done by stacking matrices
    // otherwise, these matrices should already have the good dimension, hence we perform an element-wise copy
    int h;

    if(!initStruct)
    {
        Ak.resize(hMax+1);
        Ck.resize(hMax+1);
        Bk.resize(hMax+1);
        Dk.resize(hMax+1);
        for(h=0;h<hMax+1;++h)
        {
            Ak[h] = cv::Mat(0, nX, CV_64F);
            Ck[h] = cv::Mat(0, nX, CV_64F);
            Bk[h].resize(0);
            Dk[h].resize(0);
        }
    }
   
    dbg("Loop over cascade levels")
    for(uint k=0; k<Hs.size(); ++k)
    {
        h = Hs[k];

        switch(Ts[k])
        {
            case EQUALITY:
                dbg("copy Ms into Ak and Vs into Bk")
                if(initStruct)
                {
                    dbg("Loop over rows and cols of Ms")
                    dbg("Ms[" << indM << "]->rows = " << Ms[indM]->rows)
                    dbg("Ms[" << indM << "]->cols = " << Ms[indM]->cols)
                    for(int i=0; i<Ms[indM]->rows; ++i)
                    {
                        for(int j=0; j<Ms[indM]->cols; ++j)
                        {
                            dbg("Ak[" << h << "].at<double>(" << indA[h]+i << ")")
                            Ak[h].at<double>(indA[h]+i, j) = (*(Ms[indM])).at<double>(i, j);
                        }

                        Bk[h].at<double>(indA[h]+i) = (*(Vs[indV])).at<double>(i);
                    }
                    indA[h] += Ms[indM]->rows;
                }
                else
                {
                    dbg("Ak[" << h << "].push_back(*(Ms[" << indM << "])");
                    Ak[h].push_back((*(Ms[indM])));

                    dbg("Bk[" << h << "].push_back(*(Vs[" << indV << "])");
                    Bk[h].push_back(*(Vs[indV]));
                }
                dbg("Ak[" << h << "] = " << std::endl << Ak[h] << std::endl)
                dbg("Bk[" << h << "] = " << std::endl << Bk[h] << std::endl)
                break;

            case INEQUALITY:
                dbg("copy Ms into Ck and Vs into Dk")
                if(initStruct)
                {
                    dbg("Loop over rows and cols of Ms")
                    dbg("Ms[" << indM << "]->rows = " << Ms[indM]->rows)
                    dbg("Ms[" << indM << "]->cols = " << Ms[indM]->cols)
                    for(int i=0; i<Ms[indM]->rows; ++i)
                    {
                        for(int j=0; j<Ms[indM]->cols; ++j)
                        {
                            dbg("Ck[" << h << "].at<double>(" << indC[h]+i << ")")
                            Ck[h].at<double>(indC[h]+i, j) = (*(Ms[indM])).at<double>(i, j);
                        }
                        
                        Dk[h].at<double>(indC[h]+i) = (*(Vs[indV])).at<double>(i);
                    }
                    indC[h] += Ms[indM]->rows;
                }
                else
                {
                    dbg("Ck[" << h << "].push_back(*(Ms[" << indM << "])");
                    Ck[h].push_back(*(Ms[indM]));

                    dbg("Dk[" << h << "].push_back(*(Vs[" << indV << "])");
                    Dk[h].push_back(*(Vs[indV]));
                }
                dbg("Ck[" << h << "] = " << std::endl << Ck[h] << std::endl)
                dbg("Dk[" << h << "] = " << std::endl << Dk[h] << std::endl)
                break;
        }
            
        updateIndex(Ts[k], indM, indV, indS, indSd);
    }
 
    initStruct = true;
}


/**
 * @brief 
 *
 * @param type
 * @param indM
 * @param indV
 * @param indS
 * @param 
 * @param indM
 * @param indV
 * @param indS
 * @param indSd
 */
void
hqp::updateIndex(const qpType& type, uint& indM, uint & indV, uint& indS, uint& indSd)
{ 
    dbg("")

    switch(type)
    {
        case EQUALITY:
        case INEQUALITY:
            indM++;
            indV++;
            break;
    }
}


