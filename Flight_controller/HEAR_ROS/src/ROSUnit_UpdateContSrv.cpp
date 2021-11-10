#include "HEAR_ROS/ROSUnit_UpdateContSrv.hpp"

namespace HEAR{

UpdateTrigger* ROSUnit_UpdateContSrv::registerServer(const std::string &service_topic){
    ext_trig = new UpdateTrigger;
    this->m_server = this->nh_.advertiseService(service_topic, &ROSUnit_UpdateContSrv::srv_callback, this);  
    return ext_trig;
}

bool ROSUnit_UpdateContSrv::srv_callback(hear_msgs::Update_Controller_PID::Request& req, hear_msgs::Update_Controller_PID::Response& res){
    PID_UpdateMsg msg;
    msg.param.id = (int)req.controller_parameters.id;
    msg.param.anti_windup = req.controller_parameters.pid_anti_windup;
    msg.param.en_pv_derivation = req.controller_parameters.pid_en_pv_derivation;
    msg.param.kp = req.controller_parameters.pid_kp;
    msg.param.ki = req.controller_parameters.pid_ki;
    msg.param.kd = req.controller_parameters.pid_kd;
    msg.param.kdd = req.controller_parameters.pid_kdd;
    ext_trig->UpdateCallback((UpdateMsg*)&msg);
    return true;
}

}