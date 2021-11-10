#ifndef ROSUNIT_QUATPUB_HPP
#define ROSUNIT_QUATPUB_HPP

#include "HEAR_ROS/ROSUnit_Pub.hpp"
#include "geometry_msgs/Quaternion.h"
#include "HEAR_core/Vector3D.hpp"

#include "tf2/LinearMath/Quaternion.h"

namespace HEAR{

class ROSUnitQuatPub : public ROSUnit_Pub{
public:
    ROSUnitQuatPub(ros::NodeHandle& nh, const std::string& topic_name, int idx){
        pub_ = nh.advertise<geometry_msgs::Quaternion>(topic_name, 10);
        _input_port = new InputPort<tf2::Quaternion>(idx, 0);
        id_ = idx;
    }

    TYPE getType(){ return TYPE::Float3;}
    
    void process(){
        if (_input_port != NULL){
            geometry_msgs::Quaternion msg;
            tf2::Quaternion data;
            ((InputPort<tf2::Quaternion>*)_input_port)->read(data);
            msg.x = data.x(); msg.y = data.y(), msg.z = data.z(), msg.w = data.w();
            pub_.publish(msg);
        }
    }
};

}

#endif