#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "../include/CKinematics.hpp"

#define DEBUG

#ifdef DEBUG
    #define dbg(msg) std::cout << "[DBG] " << __TIME__ << ":" << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": " << msg << std::endl;
#else
    #define dbg(msg)
#endif

    class CKinematics::CKinematicsPimpl
    {
        public:
            std::vector<cv::Mat> J2Js; // Joint-to-joint transforms
            std::vector<cv::Mat> J2Bs; // Joint-to-base transforms
            cv::Mat J;
    };

    /**
     * @brief Default constructor for CKinematics class.
     */
    CKinematics::CKinematics()
        : mPimpl(new CKinematicsPimpl)
    {
        dbg("")
        this->mPimpl->J = cv::Mat(cv::Size(3, 3), CV_64F);
    }

    CKinematics::~CKinematics()
    {
        dbg("")
        delete mPimpl;
    }

    /**
     * @brief
     */
    void
    CKinematics::Reset()
    {
        // Clear member vectors.
        dbg("")
        this->mPimpl->J2Js.erase(this->mPimpl->J2Js.begin(), this->mPimpl->J2Js.end());
        this->mPimpl->J2Bs.erase(this->mPimpl->J2Bs.begin(), this->mPimpl->J2Bs.end());
    }

    /**
     * @brief
     *
     * @param t
     */
    void
    CKinematics::addJ2J(cv::Mat t)
    {
        dbg("")
        this->mPimpl->J2Js.push_back(t);
    }

    /**
     * @brief Compute the transformation from each joint frame
     * to the base frame.
     */
    void
    CKinematics::computeJ2Bs()
    {
        dbg("")
        cv::Mat j2b = cv::Mat::eye(4, 4, CV_64F);  // 4x4 matrix

        std::vector<cv::Mat>::iterator j2j_iter;

        // Multiply out the individual DH transforms for each link.
        for(j2j_iter = this->mPimpl->J2Js.begin(); j2j_iter != this->mPimpl->J2Js.end(); ++j2j_iter)
        {
            cv::Mat j2j = (*j2j_iter);
            j2b = j2b * j2j;
            this->mPimpl->J2Bs.push_back(j2b.clone());
        }
    }

    /**
     * @brief Returns the manipulator jacobian.
     *
     * @details The manipulator Jacobian is calculated from the joint-to-base
     * transformation matrices.
     *
     * @param t
     */
    cv::Mat
    CKinematics::getJacobian()
    {
        dbg("")
        //cv::Mat J(cv::Size(3, 3), CV_64F);
        cv::Mat EEToBase = this->mPimpl->J2Bs.back();
        dbg("")

        cv::Mat j2b(cv::Size(3, 3), CV_64F);

        // oth column of J:
        // J_linear =  e_i X (O_n - O_i)

        //cv::Mat J(cv::Size(3, 3), CV_64F);
        double o_x = EEToBase.at<double>(0, 3) - 0;
        double o_y = EEToBase.at<double>(1, 3) - 0;
        double o_z = EEToBase.at<double>(2, 3) - 0;

        double z_x = 0;
        double z_y = 0;
        double z_z = 1;

        dbg("i = " << 0 << ", O = " << o_x << " " << o_y << " " << o_z);

        // ith column of J:
        // J_linear =  e_i X (O_n - O_i)
        dbg("")
        //cv::Mat J(cv::Size(3, 3), CV_64F);
        double c_x = (z_y * o_z) - (z_z * o_y); // checked
        double c_y = (z_z * o_x) - (z_x * o_z); // checked
        double c_z = (z_x * o_y) - (z_y * o_x); // checked

        dbg("cross product 0 = " << c_x << " " << c_y << " " << c_z);

        dbg("")
        //cv::Mat J(cv::Size(3, 3), CV_64F);
        this->mPimpl->J.at<double>(0, 0) = c_x;
        this->mPimpl->J.at<double>(1, 0) = c_y;
        this->mPimpl->J.at<double>(2, 0) = c_z;

        // J_angular = e_i
        //J.at<double>(3, 0) = z_x;
        //J.at<double>(4, 0) = z_y;
        //J.at<double>(5, 0) = z_z;

        dbg("")
        //cv::Mat J(cv::Size(3, 3), CV_64F);
        for(std::vector<cv::Mat>::size_type i = 1; i < this->mPimpl->J2Bs.size(); i++)
        {
            j2b = this->mPimpl->J2Bs.at(i-1);

            o_x = EEToBase.at<double>(0, 3) - j2b.at<double>(0, 3);
            o_y = EEToBase.at<double>(1, 3) - j2b.at<double>(1, 3);
            o_z = EEToBase.at<double>(2, 3) - j2b.at<double>(2, 3);

            z_x = j2b.at<double>(0, 2);
            z_y = j2b.at<double>(1, 2);
            z_z = j2b.at<double>(2, 2);

            dbg("i = " << i << ", O = " << o_x << " " << o_y << " " << o_z)

            // ith column of J:
            // J_linear =  e_i X (O_n - O_i)
            c_x = (z_y * o_z) - (z_z * o_y); // checked
            c_y = (z_z * o_x) - (z_x * o_z); // checked
            c_z = (z_x * o_y) - (z_y * o_x); // checked

            dbg("cross product " << i << " = " << c_x << " " << c_y << " " << c_z)
            this->mPimpl->J.at<double>(0, i) = c_x;
            this->mPimpl->J.at<double>(1, i) = c_y;
            this->mPimpl->J.at<double>(2, i) = c_z;

            // J_angular = e_i
            //J.at<double>(3, i) = z_x;
            //J.at<double>(4, i) = z_y;
            //J.at<double>(5, i) = z_z;
        }

        dbg("")
        //cv::Mat J(cv::Size(3, 3), CV_64F);
        return this->mPimpl->J;
    }

    /**
     * @brief Return the pseudo-inverse of a given matrix.
     *
     * @details Compute the pseudo-inverse of a given matrix using SVD.
     *
     * @param src dst
     *
     * @return
     */
    cv::Mat
    CKinematics::getPseudoInverse(cv::Mat src) const
    {
        dbg("")
        //double t1 = 0.0, t2 = 0.0;

        //double freq = cv::getTickFrequency();
        //t1 = (double)cv::getTickCount();

        cv::Mat dst = src.inv(cv::DECOMP_SVD);

        //t2 = (cv::getTickCount()-t1)/freq;
        //std::cout << "DECOMP_SVD time = " << t2 << " s" << std::endl;

        return dst;
    } // function getPseudoInverse

    /**
     * @brief Return the pseudo-inverse of the manipulator Jacobian.
     *
     * @return
     */
    cv::Mat
    CKinematics::getJacobianInverse()
    {
        dbg("")
        return this->getPseudoInverse(this->getJacobian());
    }

