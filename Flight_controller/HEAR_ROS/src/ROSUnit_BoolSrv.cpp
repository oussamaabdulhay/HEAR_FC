#include "HEAR_ROS/ROSUnit_BoolSrv.hpp"

namespace HEAR{

ROSUnit_BoolServer::ROSUnit_BoolServer(ros::NodeHandle &nh) : nh_(nh){
    ext_trig = new UpdateTrigger;
}

UpdateTrigger* ROSUnit_BoolServer::registerServer(const std::string &service_topic){
    m_server = nh_.advertiseService(service_topic, &ROSUnit_BoolServer::srv_callback, this);  
    return ext_trig;
}

bool ROSUnit_BoolServer::srv_callback(hear_msgs::set_bool::Request& req, hear_msgs::set_bool::Response& res){
    BoolMsg msg;
    msg.data = req.data;
    ext_trig->UpdateCallback((UpdateMsg*)&msg);
    return true;
}

}