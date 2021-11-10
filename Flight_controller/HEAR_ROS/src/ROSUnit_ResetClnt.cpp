#include "HEAR_ROS/ROSUnit_ResetClnt.hpp"

namespace HEAR{

ROSUnitResetClient::ROSUnitResetClient(ros::NodeHandle &nh, std::string t_name){
    m_client = nh.serviceClient<std_srvs::Empty>(t_name);
}

bool ROSUnitResetClient::process(){
    std_srvs::Empty msg;    
    return m_client.call(msg);
}

}