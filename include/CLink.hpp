#ifndef _CLINK // Has this header been included elsewhere?
#define _CLINK

#include <opencv2/opencv.hpp>

    /**
     * @class CLink
     *
     * This class models a single CLink for a robotic arm..
     */
    class CLink
    {
        friend class CLinkTest;

        public:
            CLink(double, double, double, double, double, double);
            //~CLink();
            int getID() const;
            double getDH_d() const;
            double getDH_r() const;
            double getDH_alpha() const;
            double getDH_theta() const;
            double getOffset() const;
            double getReal() const;
            cv::Mat getTransform(double);
            
        private:
            static int nextID;
            int ID;
        
            // Denavit-Haternberg Parameters
            double DH_d;
            double DH_r;
            double DH_alpha;
            double DH_theta;
            double offset;
            double real;
            cv::Mat DH_transform;
            
    }; // class CLink

#endif
