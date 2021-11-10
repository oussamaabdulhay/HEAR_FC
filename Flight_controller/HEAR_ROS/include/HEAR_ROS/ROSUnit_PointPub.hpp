#ifndef ROSUNIT_POINTPUB_HPP
#define ROSUNIT_POINTPUB_HPP

#include "HEAR_ROS/ROSUnit_Pub.hpp"
#include "geometry_msgs/Point.h"
#include "HEAR_core/Vector3D.hpp"

namespace HEAR{

class ROSUnitPointPub : public ROSUnit_Pub{
public:
    ROSUnitPointPub(ros::NodeHandle& nh, const std::string& topic_name, int idx){
        pub_ = nh.advertise<geometry_msgs::Point>(topic_name, 10);
        _input_port = new InputPort<Vector3D<float>>(idx, 0);
        id_ = idx;
    }

    TYPE getType(){ return TYPE::Float3;}
    
    void process(){
        if (_input_port != NULL){
            geometry_msgs::Point msg;
            Vector3D<float> data;
            ((InputPort<Vector3D<float>>*)_input_port)->read(data);
            msg.x = data.x; msg.y = data.y, msg.z = data.z;
            pub_.publish(msg);
        }
    }
};

}

#endif