#include "HEAR_ROS/ROSUnit_SLAM.hpp"

namespace HEAR{

ROSUnit_SLAM::ROSUnit_SLAM(ros::NodeHandle& nh, bool use_map) : nh_(nh), to_map(use_map){
    pos_inp_port = new InputPort<Vector3D<float>>(0, 0);
    ori_inp_port = new InputPort<Vector3D<float>>(0, 0);

    tf2::Matrix3x3 rot;
    rot.setIdentity();
    offset_tf = tf2::Transform(rot);
    slam_pos = tf2::Vector3(0,0,0); slam_rot.setIdentity();
}

std::vector<ExternalOutputPort<Vector3D<float>>*> ROSUnit_SLAM::registerSLAM(const std::string& t_name){
    set_offset_srv = nh_.advertiseService("/set_map_frame_offset", &ROSUnit_SLAM::srv_callback, this);
    
    odom_sub = nh_.subscribe(t_name, 10, &ROSUnit_SLAM::odom_callback, this);
    
    pos_out_port = new ExternalOutputPort<Vector3D<float>>(0);
    vel_out_port = new ExternalOutputPort<Vector3D<float>>(0);
    ori_out_port = new ExternalOutputPort<Vector3D<float>>(0);
    tf2_ros::TransformListener tf_listener(tf_Buffer);

    return std::vector<ExternalOutputPort<Vector3D<float>>*>{pos_out_port, ori_out_port, vel_out_port};
}

void ROSUnit_SLAM::connectInputs(OutputPort<Vector3D<float>>* pos_port, OutputPort<Vector3D<float>>* ori_port){
    pos_inp_port->connect(pos_port);
    ori_inp_port->connect(ori_port);
}

void ROSUnit_SLAM::odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg){
    tf2::Vector3 pos, vel;
    tf2::Matrix3x3 rot;

    geometry_msgs::PoseStamped slam_pose, tf_pose;
    slam_pose.header = odom_msg->header;
    slam_pose.pose = odom_msg->pose.pose;

    if(to_map){
        try{
            tf_Buffer.transform(slam_pose, tf_pose, ref_frame, ros::Duration(0.1));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught
        }
        pos = {tf_pose.pose.position.x, tf_pose.pose.position.y, tf_pose.pose.position.z};
        auto ori = tf2::Quaternion(tf_pose.pose.orientation.x, tf_pose.pose.orientation.y, tf_pose.pose.orientation.z, tf_pose.pose.orientation.w);
        rot.setRotation(ori);
    }
    else{
        pos = tf2::Vector3(slam_pose.pose.position.x, slam_pose.pose.position.y, slam_pose.pose.position.z);
        rot.setRotation(tf2::Quaternion(slam_pose.pose.orientation.x, slam_pose.pose.orientation.y, slam_pose.pose.orientation.z, slam_pose.pose.orientation.w));
    }
    slam_pos = offset_tf*pos;
    slam_rot = offset_tf.getBasis()*rot*(offset_tf.getBasis().transpose());

    // velocity calculation
    if(first_read == 0){
        first_read = 1;
        prevT = slam_pose.header.stamp;
        prev_pos = pos;
        vel = tf2::Vector3(0, 0, 0);
        prev_diff = vel;
    }else{
        auto _dt = (slam_pose.header.stamp - prevT).toSec();
        auto diff = (pos - prev_pos)/_dt;
        vel = diff;
        if(first_read == 1){
            first_read = 2;
            prev_diff = diff;
        }
        auto d_diff = diff - prev_diff;
        if(abs(d_diff.x()) > PEAK_THRESH || abs(d_diff.y()) > PEAK_THRESH || abs(d_diff.z()) > PEAK_THRESH){
            vel = _hold;
        }
        else{
            _hold = diff;
        }
        prev_diff = diff;
        prev_pos = pos;
        prevT = slam_pose.header.stamp;
    }
    slam_vel = offset_tf.getBasis()*vel;
    ////////////////////////

    double r, p, y;
    slam_rot.getRPY(r, p, y);
    
    pos_out_port->write(Vector3D<float>(slam_pos.x(), slam_pos.y(), slam_pos.z()));
    vel_out_port->write(Vector3D<float>(slam_vel.x(), slam_vel.y(), slam_vel.z()));
    ori_out_port->write(Vector3D<float>(r, p, y));
}

bool ROSUnit_SLAM::srv_callback(hear_msgs::set_bool::Request& req, hear_msgs::set_bool::Response& res){
    Vector3D<float> angs, trans;
    pos_inp_port->read(trans);
    ori_inp_port->read(angs);
    
    tf2::Matrix3x3 rot;
    rot.setEulerYPR(angs.z, angs.y, angs.x);
    offset_tf.setBasis(rot*(slam_rot.transpose()));
    offset_tf.setOrigin(tf2::Vector3(trans.x, trans.y, trans.z) - offset_tf.getBasis()*slam_pos);

    return true;
}

}