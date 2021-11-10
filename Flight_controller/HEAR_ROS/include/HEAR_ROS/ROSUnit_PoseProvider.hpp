#ifndef ROSUNIT_POSEPROVIDER_HPP
#define ROSUNIT_POSEPROVIDER_HPP

#include <vector>

#include "ros/ros.h"
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <hear_msgs/set_float.h>

#include "HEAR_core/Block.hpp"
#include "HEAR_core/ExternalPort.hpp"
#include "HEAR_core/Vector3D.hpp"
#include "HEAR_core/ExternalTrigger.hpp"

namespace HEAR{

class ROSUnit_PoseProvider{
private:
    ros::NodeHandle nh_;
    ros::Subscriber opti_sub, xsens_ori_sub, xsens_ang_vel_sub, xsens_free_acc_sub;
    ros::ServiceServer m_server;
    
    ExternalOutputPort<Vector3D<float>>* opti_pos_port;
    ExternalOutputPort<Vector3D<float>>* opti_ori_port;
    ExternalOutputPort<Vector3D<float>>* imu_ori_port;
    ExternalOutputPort<Vector3D<float>>* imu_acc_port;
    ExternalOutputPort<Vector3D<float>>* imu_angular_rt_port;
    void callback_opti_pose(const geometry_msgs::PoseStamped::ConstPtr& );
    void callback_ori(const geometry_msgs::QuaternionStamped::ConstPtr& );
    void callback_free_acc(const geometry_msgs::Vector3Stamped::ConstPtr& );
    void callback_angular_vel(const geometry_msgs::Vector3Stamped::ConstPtr&);
    bool srv_callback(hear_msgs::set_float::Request&, hear_msgs::set_float::Response&);
    tf2::Matrix3x3 rot_offset;
    tf2::Vector3 trans_offset;
public:
    void process(){}
    ROSUnit_PoseProvider(ros::NodeHandle& nh);
    ~ROSUnit_PoseProvider(){}
    std::vector<ExternalOutputPort<Vector3D<float>>*> registerOptiPose(std::string t_name);
    ExternalOutputPort<Vector3D<float>>* registerImuOri(std::string t_name);
    ExternalOutputPort<Vector3D<float>>* registerImuAngularRate(std::string t_name);
    ExternalOutputPort<Vector3D<float>>* registerImuAcceleration(std::string t_name);
};

}

#endif