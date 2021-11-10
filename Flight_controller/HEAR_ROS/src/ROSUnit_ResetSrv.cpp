#include "HEAR_ROS/ROSUnit_ResetSrv.hpp"

namespace HEAR{

ROSUnit_ResetServer::ROSUnit_ResetServer(ros::NodeHandle &nh) {
    ext_trig = new ResetTrigger;

}

ResetTrigger* ROSUnit_ResetServer::registerServer(const std::string &service_topic){
    m_server = nh_.advertiseService(service_topic, &ROSUnit_ResetServer::srv_callback, this);  

    return ext_trig;
}

bool ROSUnit_ResetServer::srv_callback(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res){
    ext_trig->resetCallback();
    return true;
}

}