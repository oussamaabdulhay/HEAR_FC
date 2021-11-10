#ifndef ROSUNIT_SLAM_HPP
#define ROSUNIT_SLAM_HPP

#include <vector>

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "HEAR_core/Block.hpp"
#include "HEAR_core/Vector3D.hpp"
#include "HEAR_core/ExternalPort.hpp"
#include "hear_msgs/set_bool.h"
#include "tf2/LinearMath/Transform.h"

namespace HEAR{

class ROSUnit_SLAM {
private:
    const float PEAK_THRESH = 0.35;
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub;
    ros::ServiceServer set_offset_srv;
    tf2_ros::Buffer tf_Buffer;
    std::string ref_frame = "map";
    ros::Time prevT;
    uint8_t first_read = 0;
    InputPort<Vector3D<float>>* pos_inp_port;
    InputPort<Vector3D<float>>* ori_inp_port;

    ExternalOutputPort<Vector3D<float>>* pos_out_port;
    ExternalOutputPort<Vector3D<float>>* vel_out_port;
    ExternalOutputPort<Vector3D<float>>* ori_out_port;
    tf2::Transform offset_tf;
    tf2::Vector3 slam_pos, prev_pos, slam_vel, prev_diff, _hold;
    tf2::Matrix3x3 slam_rot;    
    bool to_map = false;

    void odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg);
    bool srv_callback(hear_msgs::set_bool::Request& req, hear_msgs::set_bool::Response& res);

public:
    ROSUnit_SLAM(ros::NodeHandle& nh, bool use_map = false);
    ~ROSUnit_SLAM(){}
    std::vector<ExternalOutputPort<Vector3D<float>>*> registerSLAM(const std::string& t_name);
    void connectInputs(OutputPort<Vector3D<float>>* pos_port, OutputPort<Vector3D<float>>* ori_port);
    
};

}

#endif