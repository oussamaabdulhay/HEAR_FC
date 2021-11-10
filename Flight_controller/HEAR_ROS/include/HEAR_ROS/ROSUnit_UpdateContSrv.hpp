#ifndef ROSUNIT_UPDATECONTSRV_HPP
#define ROSUNIT_UPDATECONTSRV_HPP

#include <ros/ros.h>
#include <hear_msgs/PID_param.h>
#include <hear_msgs/Update_Controller_PID.h>
#include <string>

#include "HEAR_core/ExternalTrigger.hpp"

namespace HEAR{
class ROSUnit_UpdateContSrv {
private:
    ros::NodeHandle nh_;
    ros::ServiceServer m_server;
    UpdateTrigger* ext_trig;
    bool srv_callback(hear_msgs::Update_Controller_PID::Request&, hear_msgs::Update_Controller_PID::Response&);
public:
    ROSUnit_UpdateContSrv(ros::NodeHandle& nh) : nh_(nh) {}
    UpdateTrigger* registerServer(const std::string&);
    
};

}

#endif