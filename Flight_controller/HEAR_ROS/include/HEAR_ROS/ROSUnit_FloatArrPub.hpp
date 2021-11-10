#ifndef ROSUNIT_FLOATARRPUB_HPP
#define ROSUNIT_FLOATARRPUB_HPP

#include "HEAR_ROS/ROSUnit_Pub.hpp"
#include "std_msgs/Float32MultiArray.h"

#include <vector>

namespace HEAR{

class ROSUnitFloatArrPub : public ROSUnit_Pub{
private:
public:
    ROSUnitFloatArrPub(ros::NodeHandle& nh, const std::string& topic_name, int idx){
        pub_ = nh.advertise<std_msgs::Float32MultiArray>(topic_name, 10, true);
        _input_port = new InputPort<std::vector<float>>(idx, 0);
        id_ = idx;
    }

    TYPE getType(){ return TYPE::FloatVec;}
    void process(){
        if (_input_port != NULL){
            std_msgs::Float32MultiArray msg;
            ((InputPort<std::vector<float>>*)_input_port)->read(msg.data);
            pub_.publish(msg);
        }
    }
};

}

#endif