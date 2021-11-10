#include "HEAR_ROS/ROSUnit_UpdateMRFTsrv.hpp"

namespace HEAR{

UpdateTrigger* ROSUnit_UpdateMRFTsrv::registerServer(const std::string &service_topic){
    ext_trig = new UpdateTrigger;
    this->m_server = this->nh_.advertiseService(service_topic, &ROSUnit_UpdateMRFTsrv::srv_callback, this);  
    return ext_trig;
}

bool ROSUnit_UpdateMRFTsrv::srv_callback(hear_msgs::Update_Controller_MRFT::Request& req, hear_msgs::Update_Controller_MRFT::Response& res){
    MRFT_UpdateMsg msg;
    msg.param.id = (int)req.controller_parameters.id;
    msg.param.beta = req.controller_parameters.mrft_beta;
    msg.param.relay_amp = req.controller_parameters.mrft_relay_amp ;
    msg.param.no_switch_delay_in_ms = req.controller_parameters.mrft_no_switch_delay;
    msg.param.num_of_peak_conf_samples = req.controller_parameters.mrft_conf_samples;
    ext_trig->UpdateCallback((UpdateMsg*)&msg);
    return true;
}

}