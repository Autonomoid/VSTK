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

#include "../include/CQPSolver.hpp"

#include <iostream>
#include <vector>
#include <algorithm>

#include <opencv2/core/core.hpp>

template < class T >
std::ostream& operator << (std::ostream& os, const std::vector<T>& v)
{
    os << "[";
    typename std::vector<T>::const_iterator ii;
    for(ii = v.begin(); ii != v.end()-1; ++ii)
    {
        os << *ii << ",";
    }
    os << *ii << "]";
    return os;
}

namespace vs
{

/**
 * @brief
 *
 * @param Q
 * @param r
 * @param A
 * @param b
 * @param C
 * @param d
 * @param x
 */
CQPSolver::CQPSolver(const cv::Mat& _Q, const cv::Mat& _r, const cv::Mat& _A, const cv::Mat _b, const cv::Mat& _C, const cv::Mat& _d)
    : Q(_Q), r(_r), A(_A), b(_b), C(_C), d(_d),
      nAct(0),
      ineqIndex(-1), ineqCount(0), maxViolation(-1),
      zeroThresh(1e-6), errorBest(-1), errorCurrent(-1),
      epsilon(0.0)
{
    dbg("")
    this->init();
}


/**
 * @brief
 *
 * @return
 */
cv::Mat
CQPSolver::getSolution() const
{
    return this->soln.clone();
}


/**
 * @brief
 *
 * @return
 */
void
CQPSolver::setEpsilon(double epsilon_)
{
    this->epsilon = epsilon_;
}


/**
 * @brief
 */
void
CQPSolver::init()
{
    Q_cols = Q.cols;
    Q_rows = Q.rows;

    A_cols = A.cols;
    A_rows = A.rows;

    C_cols = C.cols;
    C_rows = C.rows;

    r_cols = r.cols;
    r_rows = r.rows;

    b_cols = b.cols;
    b_rows = b.rows;

    d_cols = d.cols;
    d_rows = d.rows;

    dbg("")

    // Create an initial active set. One element per constraint, each set to false.
    for(uint i=0; i<C_rows; ++i)
    {
        activeSet.push_back(false);
    }
    dbg2("Initialialized active set." << std::endl)

    // Assume at least 5 previous active sets will be stored.
    this->prevActiveSets.reserve(5);

}


/**
 * @brief
 *
 * @return
 */
int
CQPSolver::checkMatrixDims()
{
    if(Q_cols != A_cols)
    {
        dbg("")
        std::cout << "Wrong matrix dimensions:" << std::endl
                  << "Q_cols = " << Q_cols << ", A_cols = " << A_cols << std::endl;
        return -1;
    }

    if(Q_cols != C_cols)
    {
        dbg("")
        std::cout << "Wrong matrix dimensions:" << std::endl
                  << "Q_cols = " << Q_cols << ", C_cols = " << C_cols << std::endl;
        return -1;
    }

    if(A_rows != b_rows)
    {
        dbg("")
        std::cout << "Wrong matrix dimensions:" << std::endl
                  << "A_rows = " << A_rows << ", b_rows = " << b_rows << std::endl;
        return -1;
    }

    if(C_rows != d_rows)
    {
        dbg("")
        std::cout << "Wrong matrix dimensions:" << std::endl
                  << "C_rows = " << C_rows << ", d_rows = " << d_rows << std::endl;
        return -1;
    }

    if(Q_rows != r_rows)
    {
        dbg("")
        std::cout << "Wrong matrix dimensions:" << std::endl
                  << "Q_rows = " << Q_rows << ", r_rows = " << r_rows << std::endl;
        return -1;
    }

    // Everything is OK.
    return 0;

}


/**
 * @brief
 *
 * @return
 */
int
CQPSolver::checkTrivialSoln()
{
    dbg("")

    // Check for trivial solution (i.e. only inequalities)
    // r = 0,
    // b empty or null,
    // d > 0
    // --> soln = 0

    double d_min = -1;
    cv::minMaxLoc(d, &d_min);

    if(cv::norm(r, cv::NORM_L2) == 0 &&
            (d.rows == 0 || d_min >= 0) &&
            (b.rows == 0 || cv::norm(b, cv::NORM_L2) == 0))
    {
        std::cout << "Trivial solution"	 << std::endl;
        this->soln = cv::Mat::zeros(int(Q_cols), 1, CV_64F);

        return 0;
    }

    dbg("")

    // No trivial solution found.
    return -1;
}


/**
 * @brief
 */
void
CQPSolver::buildMatrices()
{
    tempSoln = cv::Mat(Q_cols, 1, CV_64F);
    KKT = cv::Mat::zeros(Q_rows+A_rows+nAct, Q_cols+A_rows+nAct, cv::DataType<double>::type);
    v = cv::Mat::zeros(Q_rows+A_rows+nAct, 1, cv::DataType<double>::type);

    for(uint j=0; j<Q_cols; ++j)
    {
        // Q and r
        for(uint i=0; i<Q_rows; ++i)
        {
            KKT.at<double>(i, j) = Q.at<double>(i, j);
            v.at<double>(i) = r.at<double>(i);
        }

        // A and b
        for(uint k=0; k<A_rows; ++k)
        {
            KKT.at<double>(Q_rows+k, j) = KKT.at<double>(j, Q_cols+k) = A.at<double>(k, j);
            v.at<double>(Q_rows+k) = b.at<double>(k);
        }

        // active set from C and d
        ineqCount = 0;
        for(uint m=0; m<C_rows; ++m)
        {
            if(activeSet[m])
            {
                KKT.at<double>(Q_rows+A_rows+ineqCount, j) = KKT.at<double>(j, Q_cols+A_rows+ineqCount) = C.at<double>(m, j);
                v.at<double>(Q_rows+A_rows+ineqCount) = d.at<double>(m);
                ineqCount++;
            }
        }
    }
}


/**
 * @brief
 */
void
CQPSolver::computeRegularizedSoln()
{
    cv::Mat I = cv::Mat::eye(KKT.rows, KKT.rows, cv::DataType<double>::type);
    cv::Mat temp = KKT + (epsilon * I);
    dbg("temp = " << std::endl << temp)

    X = temp.inv(cv::DECOMP_SVD) * v;

    // cv::Rect(x, y, width, height)
    //tempSoln = X(cv::Rect(0, 0, 1, Q_cols));
    for(int i=0; i<tempSoln.rows; ++i)
    {
        tempSoln.at<double>(i) = X.at<double>(i);
    }

    std::cout << "tempSoln = " << std::endl
              << tempSoln << std::endl;
}


/**
 * @brief
 */
void
CQPSolver::findMostViolatedConstraint()
{
    dbg("")

    maxViolation = 0.0;
    for(uint i=0; i<C_rows; ++i)
    {
        dbg("i = " << i)
        dbg("C.row(i) = " << std::endl << C.row(i) << std::endl)
        dbg("d.at(i) = " << std::endl << d.at<double>(i) << std::endl)

        cv::Mat temp = (C.row(i) * tempSoln) - d.at<double>(i);
        double violation = temp.at<double>(0);

        if(violation > (maxViolation + zeroThresh))
        {
            maxViolation = violation;
            ineqIndex = i;
        }

        dbg("Total violation for C row " << i << " = " << violation)
        dbg("Current maximum +ve violation = " << maxViolation)
        dbg("Most violated constraint is C row " << ineqIndex)
    }
}


/**
 * @brief
 */
void
CQPSolver::checkSolution()
{
    dbg("")

    errorCurrent = cv::norm(Q*tempSoln - r);
    dbg("errorCurrent = " << errorCurrent)
    if(errorBest == -1 || errorCurrent < errorBest)
    {
        errorBest = errorCurrent;
        this->soln = tempSoln;
        dbg("errorBest = " << errorBest)
    }
}


/**
 * @brief
 */
void
CQPSolver::findLeastViolatedActiveConstraint()
{
    dbg("")

    ineqCount = 0;
    // Loop over all constraints.
    for(uint i=0; i<C_rows; ++i)
    {
        // If the ith constraint is active...
        if(activeSet[i])
        {
            ineqCount++;

            double violation = X.at<double>(Q_cols+A_rows+ineqCount);
            if(violation < maxViolation)
            {
                maxViolation = violation;
                ineqIndex = i;
            }
        }
    }
}


/**
 * @brief
 *
 * @return
 */
void
CQPSolver::checkPrevActiveSets()
{
    dbg("")

    ineqIndex = -1;
    ineqCount = 0;

    // Loop over the active set
    for(uint i=0; i<prevActiveSets.size() && ineqCount != C_rows; ++i)
    {
        ineqCount = 0;

        // Loop over the set all constraints
        for(uint j=0; j<C_rows; ++j)
        {
            // Compare the jth element of the ith prev active set
            // with the jth element of the current active set.
            if(prevActiveSets[i][j] == activeSet[j])
                ineqCount++;
        }
    }

    dbg("")
}


/**
 * @brief
 */
int
CQPSolver::solve()
{
    // Check dimensions of input matrices.
    // If they don't match then quit.
    if(this->checkMatrixDims() == -1)
        return -1;

    // Check for existence of trivial solution.
    // If one exists then quit.
    if(this->checkTrivialSoln() == 1)
        return 0;

    // Main loop for the active set algorithm
    int iterations = 1;
    while(true)
    {
        // Store current active set.
        prevActiveSets.push_back(activeSet);

        // Build KKT matrix and V matrix.
        this->buildMatrices();
        dbg2("KKT = " << std::endl << KKT << std::endl)
        dbg2("v = " << std::endl << v << std::endl)

        // Compute regularized solution.
        this->computeRegularizedSoln();
        dbg2("Full solutions (X) = " << std::endl << X << std::endl)
        dbg2("Partial solution (x) = " << std::endl << tempSoln << std::endl)

        // Find most violated inequality constraint.
        this->findMostViolatedConstraint();

        // If maximum inequality constraint
        // violation > 0 then add it to
        // the active set.
        //if(maxViolation > 0.000001)
        if(maxViolation != 0.0)
        {
            nAct++;
            dbg("++nAct = " << nAct)

            activeSet[ineqIndex] = true;
            dbg("Most violated constraint (index) = " << ineqIndex)

            dbg("maxViolation = " << maxViolation)
        }

        // Else if no violated inequality constraints then...
        else
        {
            dbg("")

            // Check if this solution better than the current best solution?
            this->checkSolution();

            dbg("")

            // Remove the least violated from the active set.
            this->findLeastViolatedActiveConstraint();

            dbg("")

            if(maxViolation != 0.0)
            {
                activeSet[ineqIndex] = false;
                nAct--;
                dbg("maxViolation = " << maxViolation)
                dbg("--nAct = " << nAct)
                dbg("Least violated constraint (index) = " << ineqIndex)
            }
            // Exit while loop.
            else
            {
                break;
            }
        }

        // Check if this particular active set
        // has been tried previously.
        this->checkPrevActiveSets();

        dbg("")

        // If it has been tried previously then
        // exit the while loop.
        if(ineqCount == C_rows)
        {
            dbg("ineqCount == C_rows")
            break;
        }

        dbg("")

        ++iterations;
    } // while

    dbg2("[+] Maximum constraint violation converged to <= " << epsilon
    << " after " << iterations << " iterations.")
    return 0;
} // solve()

} // namespace
