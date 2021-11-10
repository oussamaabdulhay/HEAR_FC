#ifndef ROSUNIT_UPDATEMRFTSRV_HPP
#define ROSUNIT_UPDATEMRFTSRV_HPP

#include <ros/ros.h>
#include <hear_msgs/PID_param.h>
#include <hear_msgs/Update_Controller_MRFT.h>
#include <string>

#include "HEAR_core/ExternalTrigger.hpp"

namespace HEAR{
class ROSUnit_UpdateMRFTsrv {
private:
    ros::NodeHandle nh_;
    ros::ServiceServer m_server;
    UpdateTrigger* ext_trig;
    bool srv_callback(hear_msgs::Update_Controller_MRFT::Request&, hear_msgs::Update_Controller_MRFT::Response&);
public:
    ROSUnit_UpdateMRFTsrv(ros::NodeHandle& nh) : nh_(nh) {}
    UpdateTrigger* registerServer(const std::string&);
    
};

}

#endif