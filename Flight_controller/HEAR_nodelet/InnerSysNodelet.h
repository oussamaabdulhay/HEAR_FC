#include "ros/ros.h"
#include "nodelet/nodelet.h"

#include "HEAR_ROS/RosSystem.hpp"
#include "HEAR_ROS/ROSUnit_PoseProvider.hpp"
#include "HEAR_ROS/RosUnit_MRFTSwitchSrv.hpp"

#include "FC_util.hpp"

namespace HEAR
{
class InnerSysNodelet : public nodelet::Nodelet{

public:
    InnerSysNodelet() = default;
    virtual ~InnerSysNodelet();

private:
    const int FREQUENCY = 200;
    const float YAW_SAT_VALUE = 0.87;

    virtual void onInit();
    
    RosSystem* inner_sys;
    ROSUnit_PoseProvider* providers; 
    ROSUnit_MRFTSwitchSrv* trig_srv_roll;
    ROSUnit_MRFTSwitchSrv* trig_srv_pitch;
   
};
    
} 