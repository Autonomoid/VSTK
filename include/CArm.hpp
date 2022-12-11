#ifndef CARM_H
#define CARM_H

#include <ros/ros.h>

#include "CArmModel.hpp"
#include "CArmInterface.hpp"

namespace vs
{

class CArm
{
public:
   CArm(const ros::NodeHandle& nodeHandle);
   ~CArm(){}
   void setNDOF(const uint nDOF);
   boost::scoped_ptr<CArmModel> mModel;
   boost::scoped_ptr<CArmInterface> mInterface;
};

} // namespace vs

#endif
