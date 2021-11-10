#include "HEAR_ROS/RosSystem.hpp"
#include "HEAR_ROS/ROSUnit_SLAM.hpp"

using namespace HEAR;

int main(int argc, char **argv){
    ros::init(argc, argv, "slam_node");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    RosSystem* sys =  new RosSystem(nh, pnh, 30, "SLAM sys");    

    auto providers_slam = new ROSUnit_SLAM(nh);
    auto sub_ori = sys->createSub(TYPE::Float3 ,"/Inner_Sys/body_ori");
    auto sub_pos = sys->createSub(TYPE::Float3 ,"/opti/pos");
    providers_slam->connectInputs(sub_pos->getOutputPort<Vector3D<float>>(), sub_ori->getOutputPort<Vector3D<float>>());
    auto slam_port = providers_slam->registerSLAM("/zedm/zed_node/odom");

    auto pos_port = sys->createExternalInputPort<Vector3D<float>>("Pos_port");
    auto vel_port = sys->createExternalInputPort<Vector3D<float>>("Vel_port");
    auto ori_port = sys->createExternalInputPort<Vector3D<float>>("Ori_port");
    sys->connectExternalInput(pos_port, slam_port[0]);
    sys->connectExternalInput(ori_port, slam_port[1]);
    sys->connectExternalInput(vel_port, slam_port[2]);

    
    // connect publishers to KF output ports
    sys->createPub<Vector3D<float>>(TYPE::Float3, "/slam/pos", ((Block*)pos_port)->getOutputPort<Vector3D<float>>(0));
    sys->createPub<Vector3D<float>>(TYPE::Float3, "/slam/vel", ((Block*)vel_port)->getOutputPort<Vector3D<float>>(0));
    sys->createPub<Vector3D<float>>(TYPE::Float3, "/slam/ori", ((Block*)ori_port)->getOutputPort<Vector3D<float>>(0));

    sys->start();
    ros::spin();

}