#ifndef ROSUNIT_FLOATSUB_HPP
#define ROSUNIT_FLOATSUB_HPP

#include "HEAR_ROS/ROSUnit_Sub.hpp"
#include "std_msgs/Float32.h"

namespace HEAR{

class ROSUnitFloatSub : public ROSUnit_Sub{
private:
    void callback(const std_msgs::Float32::ConstPtr&);
public:
    ROSUnitFloatSub(ros::NodeHandle& nh, const std::string& topic, int idx);
    TYPE getType(){ return TYPE::Float;}
};

ROSUnitFloatSub::ROSUnitFloatSub(ros::NodeHandle& nh, const std::string& topic, int idx){
    sub_ = nh.subscribe<std_msgs::Float32>(topic, 10, &ROSUnitFloatSub::callback, this);
    _output_port = new OutputPort<float>(idx, 0);
    
    ((OutputPort<float>*)_output_port)->write(0);
    id_ = idx;
}
void ROSUnitFloatSub::callback(const std_msgs::Float32::ConstPtr& msg){
    ((OutputPort<float>*)_output_port)->write(msg->data);
}

}
#endif