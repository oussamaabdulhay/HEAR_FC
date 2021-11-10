#ifndef ROSUNIT_RESETCLNT
#define ROSUNIT_RESETCLNT

#include <ros/ros.h>
#include <std_srvs/Empty.h>

namespace HEAR{

class ROSUnitResetClient {
private:
    ros::ServiceClient m_client;
public:
    ROSUnitResetClient(ros::NodeHandle&, std::string);
    bool process();
};

}

#endif