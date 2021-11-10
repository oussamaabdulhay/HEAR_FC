#ifndef ROSUNIT_HEARTBEAT_HPP
#define ROSUNIT_HEARTBEAT_HPP

#include <ros/ros.h>
#include "hear_msgs/set_bool.h"
#include "std_msgs/Empty.h"

namespace HEAR{

class ROSUnit_Heartbeat {
private:
    ros::Timer _timer;
    ros::Subscriber _sub_heartbeat;
    ros::ServiceClient _srv_clnt;
    ros::Duration _interval;
    ros::Time _last_heartbeat;
public:
    ros::Duration interval;
    ROSUnit_Heartbeat(ros::NodeHandle &nh, const ros::Duration &period, std::string topic){
        _sub_heartbeat = nh.subscribe("/heartbeat", 10, &ROSUnit_Heartbeat::heartbeatCb, this );
        _timer = nh.createTimer(period, &ROSUnit_Heartbeat::timerCb , this);
        _srv_clnt = nh.serviceClient<hear_msgs::set_bool>(topic);
        _last_heartbeat = ros::Time::now();
        _interval = period;
    }
    ROSUnit_Heartbeat(ros::NodeHandle &nh, const ros::Duration &period) : ROSUnit_Heartbeat(nh, period, "/arm"){}
    void timerCb(const ros::TimerEvent &event){
        if ((event.current_real - _last_heartbeat) > (_interval*2)){
            hear_msgs::set_bool srv;
            srv.request.data = false;
            _srv_clnt.call(srv);
        }
    }
    void heartbeatCb(const std_msgs::Empty::ConstPtr msg){
        _last_heartbeat = ros::Time::now();
    }

};

}
#endif