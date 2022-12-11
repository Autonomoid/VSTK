#include "../include/CHQPSolver.hpp"
#include "../include/CQPSolver.hpp"
#include "../include/utils.hpp"

//#define DEBUG
#ifdef DEBUG
#define dbg(msg) std::cout << "[DBG] " << __TIME__ << ":" << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": " << msg << std::endl;
#else
#define dbg(msg)
#endif

//#define DEBUG2
#ifdef DEBUG2
#define dbg2(msg) std::cout << "[DBG_2] " << __TIME__ << ":" << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": " << msg << std::endl;
#else
#define dbg2(msg)
#endif

namespace vs
{

/**
 * @brief
 */
CHQPSolver::CHQPSolver()
: initStruct(false), hMax(0), rho(0.05), nX(0),
  thresh0(1e-6), indE(0), indM(0), indV(0), indS(0), indSd(0), indAct(0),
  indA(std::vector<uint>(hMax+1, 0)),
  indC(std::vector<uint>(hMax+1, 0)),
  epsilon(0.000001)
{
}


void
CHQPSolver::setEpsilon(double epsilon_)
{
    this->epsilon = epsilon_;
}


void
CHQPSolver::updateXdim(const uint& colM)
{
    if(colM != 0 && colM != nX)
    {
        if(nX == 0)
        {
            nX = colM;
        }
    }
    initStruct = false;
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
CHQPSolver::addInequality(cv::Mat& C, cv::Mat& d, const uint& h)
{
    //std::cout << "[+] Storing inequality constraint for hierarchy level " << h << "." << std::endl;
    updateXdim(C.cols);

    Ms.push_back(&C);
    Vs.push_back(&d);
    Ts.push_back(INEQUALITY);
    Hs.push_back(h);

    // Keep track of the number of hierarchy levels
    if (uint(hMax) < h)
        hMax = h;
}


void
CHQPSolver::addEquality(cv::Mat& A, cv::Mat& b, const uint& h)
{
    //std::cout << "[+] Storing equality constraint for hierarchy level " << h << "." << std::endl;
    updateXdim(A.cols);

    Ms.push_back(&A);
    Vs.push_back(&b);
    Ts.push_back(EQUALITY);
    Hs.push_back(h);

    // Keep track of the number of hierarchy levels
    if (uint(hMax) < h)
        hMax = h;

    // Keep track of the number of equality constraints.
    indE++;
}


/**
 * @brief
 *
 * @param solution
 */
int
CHQPSolver::solveCascade(cv::Mat& solution)
{
    std::cout << "\n[+] Started solving HQP." << std::endl;

    // Initialize the constraint matrices Ak, Bk, Ck, Dk.
    initKMatrices();   

    // Initialize the meta-matrices AA, BB, CC, DD.
    this->AA = cv::Mat(0, nX, CV_64F);
    this->CC = cv::Mat(0, nX, CV_64F);
    this->BB = cv::Mat(0, 0, CV_64F);
    this->DD = cv::Mat(0, 0, CV_64F);

    // Initialize the canonical matrices.
    cv::Mat Q, A, C, Iw;
    cv::Mat r, b, d, X, x;
    uint nW = 0;

    cv::Mat tempA;
    cv::Mat tempB;

    // Loop over all hierarchy levels (QPs)
    for(int h=0; h<hMax+1; ++h)
    {
        std::cout << "\n[+] Entering hierarchy level " << h << std::endl;
        nW = Ck[h].rows;

        if(nW != 0 || Bk[h].rows != 0)
        {
            //std::cout << "[+] Building canonical matrices (Q, r, A, b, C, d)." << std::endl;
            Iw = cv::Mat::eye(nW, nW, CV_64F);

            // Q
            vs::utils::hconcat(Ak[h], cv::Mat::zeros(Ak[h].rows, nW, Ak[h].type()), tempA);
            vs::utils::hconcat(cv::Mat::zeros(nW, Ak[h].cols, Ak[h].type()), Iw, tempB);
            vs::utils::vconcat(tempA, tempB, Q);

            // r
            vs::utils::vconcat(Bk[h], cv::Mat::zeros(nW, 1, CV_64F), r);

            // r (2)
            if(Q.cols != 0 && Q.rows != 0 && r.rows != 0)
            {
                r = Q.t()*r;
            }

            // q (2)
            if(Q.cols != 0 && Q.rows != 0)
            {
                Q = Q.t()*Q;
            }

            if(AA.rows > 0)
            {
                // A = [AA, 0; 0, 0]
                vs::utils::hconcat(AA, cv::Mat::zeros(AA.rows, nW, AA.type()), tempA);
                tempB = cv::Mat::zeros(nW, 2*AA.cols, AA.type());
                vs::utils::vconcat(tempA, tempB, A);

                // b = [BB; 0]
                vs::utils::vconcat(BB, cv::Mat::zeros(nW, 1, BB.type()), b);
            }
            else
            {
                A = cv::Mat(0, nX+nW, CV_64F);
                b.resize(0);
            }

            if(CC.rows + nW > 0)
            {
                // C = [CC, 0; Ck[h], -Iw]
                vs::utils::hconcat(CC, cv::Mat::zeros(CC.rows, nW, CC.type()), tempA);
                vs::utils::hconcat(Ck[h], -Iw, tempB);
                vs::utils::vconcat(tempA, tempB, C);

                // d = [DD; Dk[h]]
                vs::utils::vconcat(DD, Dk[h], d);
            }
            else
            {
                C = cv::Mat(0, nX+nW, CV_64F);
                d.resize(0);
            }

            // Initialize QP solver.

            // Validate the inputs to the solver:
            if((A.cols == 0) || (b.rows == 0))
            {
                dbg("No equality constraints.")
                //A = cv::Mat::zeros(1, Q.cols, cv::DataType<double>::type);
                //b = cv::Mat::zeros(A.rows, 1, cv::DataType<double>::type);
            }

            dbg("Input to QP solver for hierachy level " << h)
            dbg("Q = " << Q)
            dbg("r = " << r)
            dbg("A = " << A)
            dbg("b = " << b)
            dbg("C = " << C)
            dbg("d = " << d)

            dbg("")

            vs::CQPSolver solver(Q, r, A, b, C, d);

            dbg("")

            solver.setEpsilon(this->epsilon);

            //std::cout << "[+] Launching QP solver." << std::endl;
            int res = solver.solve();

            dbg("")

            if(res == -1)
            {
                //std::cout << "Quitting due to QPSolver error." << std::endl;
                exit(1);
            }

            dbg("")

            // Extract the current solution.
            X = solver.getSolution();

            // If no solution
            if(X.rows == 0)
            {
                std::cout << "No solution found (setting all joint velocities --> zero)" << std::endl;
                x = cv::Mat::zeros(nX, 1, CV_64F);
            }
            else{
                x = X(cv::Rect(0, 0, 1, nX));
            }

            //dbg2("[+] QP solver returned the primal solution: x = " << std::endl << x)

            // Unless we are at the last iteration, update AA, BB, CC, DD
            // and continue.
            if(h != hMax)
            {
                //std::cout << "[+] Updating meta-matrices." << std::endl;
                AA.push_back(Ak[h]);

                cv::Mat temp = Ak[h]*x;
                BB.push_back(temp);

                for(uint i=0; i<nW; ++i)
                {
                    cv::Mat temp = Ck[h].row(i)*x;

                    double t = temp.at<double>(0);
                    if(t < Dk[h].at<double>(i) + thresh0)
                    {
                        // Ensure that inequalities stay inequalities.
                        CC.push_back(Ck[h].row(i));
                        DD.push_back(Dk[h].at<double>(i));
                    }
                    else
                    {
                        // Violated inequalities become equalities")
                        AA.push_back(Ck[h].row(i));
                        BB.push_back(temp);
                    }
                }

                // end update matrices
            } // if
        }
        else
        {
            //std::cout << "[+] No constraints were specified for this hierarchy level." << std::endl;
        }
    } // for (h)

    solution = xPrev = x;
    //std::cout << "[+] Finished solving HQP." << std::endl;

    return 0;
}


/**
 * @brief
 */
void
CHQPSolver::initKMatrices()
{
    //std::cout << "[+] Building constraint matrices (Ak, Bk, Ck, Dk)." << std::endl;

    // initialize the matrices Ak, Bk, Ck, Dk from stored values
    // if it is the first time, this is done by stacking matrices
    // otherwise, these matrices should already have the good dimension,
    // hence we perform an element-wise copy
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

    // Loop over all constraints.
    for(uint k=0; k<Hs.size(); ++k)
    {
        // Which QP are these constraints for --> h
        h = Hs[k];

        // Switch on the type of constraint e.g.
        // equality or inequality.
        switch(Ts[k])
        {
            case EQUALITY:
                // Copy Ms into Ak and Vs into Bk
                if(initStruct)
                {
                    // Loop over rows and cols of Ms
                    for(int i=0; i<Ms[indM]->rows; ++i)
                    {
                        for(int j=0; j<Ms[indM]->cols; ++j)
                        {
                            Ak[h].at<double>(indA[h]+i, j) = (*(Ms[indM])).at<double>(i, j);
                        }

                        Bk[h].at<double>(indA[h]+i) = (*(Vs[indV])).at<double>(i);
                    }
                    indA[h] += Ms[indM]->rows;
                }
                else
                {
                    Ak[h].push_back((*(Ms[indM])));
                    Bk[h].push_back(*(Vs[indV]));
                }
                break;

            case INEQUALITY:
                // Copy Ms into Ck and Vs into Dk
                if(initStruct)
                {
                    // Loop over rows and cols of Ms
                    for(int i=0; i<Ms[indM]->rows; ++i)
                    {
                        for(int j=0; j<Ms[indM]->cols; ++j)
                        {
                            Ck[h].at<double>(indC[h]+i, j) = (*(Ms[indM])).at<double>(i, j);
                        }

                        Dk[h].at<double>(indC[h]+i) = (*(Vs[indV])).at<double>(i);
                    }

                    // Keep track of number of inequality constraints.
                    indC[h] += Ms[indM]->rows;
                }
                else
                {
                    Ck[h].push_back(*(Ms[indM]));
                    Dk[h].push_back(*(Vs[indV]));
                }
                break;
        }

        updateIndex(Ts[k], indM, indV, indS, indSd);
    }
    // end initialization of Ak, Bk, Ck, Dk
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
CHQPSolver::updateIndex(const qpType& type, uint& indM, uint & indV, uint& indS, uint& indSd)
{
    switch(type)
    {
        case EQUALITY:
        case INEQUALITY:
            indM++;
            indV++;
            break;
    }
}

} // namespace
