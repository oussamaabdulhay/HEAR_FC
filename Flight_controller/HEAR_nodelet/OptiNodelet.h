#include "ros/ros.h"
#include "nodelet/nodelet.h"

#include "HEAR_ROS/RosSystem.hpp"
#include "HEAR_ROS/ROSUnit_PoseProvider.hpp"

namespace HEAR
{
class OptiNodelet : public nodelet::Nodelet{

public:
    OptiNodelet() = default;
    virtual ~OptiNodelet();

private:
    const int FREQUENCY = 90;
    virtual void onInit();
    
    RosSystem* sys;
    ROSUnit_PoseProvider* providers;    
};
    
} 