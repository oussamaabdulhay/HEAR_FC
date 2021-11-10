#ifndef ROSUNIT_QUATSUB_HPP
#define ROSUNIT_QUATSUB_HPP

#include "HEAR_ROS/ROSUnit_Sub.hpp"
#include "HEAR_core/Vector3D.hpp"
#include "geometry_msgs/Quaternion.h"

#include "tf2/LinearMath/Quaternion.h"

namespace HEAR{
class ROSUnitQuatSub : public ROSUnit_Sub{
private:
    void callback(const geometry_msgs::Quaternion::ConstPtr& msg);
public:
    ROSUnitQuatSub (ros::NodeHandle& nh, const std::string& topic, int idx);
    TYPE getType(){return TYPE::Float3;}
};

ROSUnitQuatSub::ROSUnitQuatSub (ros::NodeHandle& nh, const std::string& topic, int idx){
    sub_ = nh.subscribe<geometry_msgs::Quaternion>(topic, 10, &ROSUnitQuatSub::callback, this);
    _output_port = new OutputPort<tf2::Quaternion>(idx, 0);
    id_ = idx;
}

void ROSUnitQuatSub::callback(const geometry_msgs::Quaternion::ConstPtr& msg){
    tf2::Quaternion data(msg->x, msg->y, msg->z, msg->w);
    ((OutputPort<tf2::Quaternion>*)_output_port)->write(data);
}

}

#endif