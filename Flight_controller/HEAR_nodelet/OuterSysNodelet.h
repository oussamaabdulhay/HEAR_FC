#include "ros/ros.h"
#include "nodelet/nodelet.h"

#include "HEAR_ROS/RosSystem.hpp"
#include "HEAR_ROS/RosUnit_MRFTSwitchSrv.hpp"
#include "FC_util.hpp"
namespace HEAR
{
class OuterSysNodelet : public nodelet::Nodelet{

public:
    OuterSysNodelet() = default;
    virtual ~OuterSysNodelet();

private:
    // const float ACC_REF_K_X = 0.0416;
    // const float ACC_REF_K_Y = 0.0416;
    const float SAT_XY_VALUE = 0.87;
    const float SAT_Z_VALUE = 22;
    const float BIAS_Z_VALUE = 9.8;
    const int FREQUENCY = 200;
    virtual void onInit();
    
    RosSystem* outer_sys;    
    ROSUnit_MRFTSwitchSrv* trig_srv_x;
    ROSUnit_MRFTSwitchSrv* trig_srv_z;
    ROSUnit_MRFTSwitchSrv* trig_srv_y;

};
    
} 