
#ifndef ROSUNIT_RESETSRV_HPP
#define ROSUNIT_RESETSRV_HPP

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <string>

#include "HEAR_core/ExternalTrigger.hpp"

namespace HEAR{
class ROSUnit_ResetServer {
private:
    ros::NodeHandle nh_;
    ResetTrigger* ext_trig;
    ros::ServiceServer m_server;
    bool srv_callback(std_srvs::EmptyRequest&, std_srvs::EmptyResponse&);
public:
    ROSUnit_ResetServer(ros::NodeHandle&);
    ResetTrigger* registerServer(const std::string&);
    
 
};

}

#endif