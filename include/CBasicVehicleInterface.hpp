#ifndef CBasicVehicleInterface_H
#define CBasicVehicleInterface_H

#include <vector>
#include <ros/ros.h>

#include "IRobotInterface.hpp"
#include "utils.hpp"

class CBasicVehicleInterface : IRobotInterface
{
  public:
    CRobotController();
    virtual void init(ros::NodeHandle&) = 0;
    virtual void drive() = 0;
    virtual void setVelocity(std::vector<double>&) = 0;
    virtual void setPose(std::vector<double>&) = 0;
    virtual void setPose(cv::Mat&) = 0;
    void setNDof(int);
    void stop();
    void setHomePose(std::vector<double>&);
    void savePose(int);
    void loadPose(int);
    void setLimits(double, double);

    std::vector<double> getPose() const;
    std::vector<double> getLowerLimits() const;
    std::vector<double> getUpperLimits() const;

    int ndof;
    bool NDOF_INIT;
    std::vector<double> home_pose;  
    std::vector<double> pose;  
    std::vector<double> savedPoses[10];
    std::vector<double> velocity;
    std::vector<double> stopCommand;
    std::vector<double> lowerLimits;
    std::vector<double> upperLimits;
};

#endif
