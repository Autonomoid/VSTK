#ifndef _CKinematics // Has this header been included elsewhere?
#define _CKinematics

#include <opencv2/opencv.hpp>

    /**
     * @class CKinematics
     *
     * This class provides forwards and inverse CKinematics.
     */
    class CKinematics
    {
        friend class CKinematicsTest;

        public:
            CKinematics();
            ~CKinematics();
            void Reset();
            void addJ2J(cv::Mat t);
            void computeJ2Bs();
            cv::Mat getJacobian();
            cv::Mat getPseudoInverse(cv::Mat src) const;
            cv::Mat getJacobianInverse();
            
        private:
            class CKinematicsPimpl;
            CKinematicsPimpl* mPimpl;

    }; // class CKinematics

#endif
