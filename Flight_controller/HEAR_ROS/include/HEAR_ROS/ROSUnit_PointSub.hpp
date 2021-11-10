#ifndef ROSUNIT_POINTSUB_HPP
#define ROSUNIT_POINTSUB_HPP

#include "HEAR_ROS/ROSUnit_Sub.hpp"
#include "HEAR_core/Vector3D.hpp"
#include "geometry_msgs/Point.h"

namespace HEAR{
class ROSUnitPointSub : public ROSUnit_Sub{
private:
    void callback(const geometry_msgs::Point::ConstPtr& msg);
public:
    ROSUnitPointSub (ros::NodeHandle& nh, const std::string& topic, int idx);
    TYPE getType(){return TYPE::Float3;}
};

ROSUnitPointSub::ROSUnitPointSub (ros::NodeHandle& nh, const std::string& topic, int idx){
    sub_ = nh.subscribe<geometry_msgs::Point>(topic, 10, &ROSUnitPointSub::callback, this, ros::TransportHints().tcpNoDelay());
    _output_port = new OutputPort<Vector3D<float>>(idx, 0);
    id_ = idx;
}

void ROSUnitPointSub::callback(const geometry_msgs::Point::ConstPtr& msg){
    Vector3D<float> data(msg->x, msg->y, msg->z);
    ((OutputPort<Vector3D<float>>*)_output_port)->write(data);
}

}

#endif