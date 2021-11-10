#include "HEAR_ROS/RosSystem.hpp"
#include "HEAR_ROS/ROSUnit_PoseProvider.hpp"


using namespace HEAR;

int main(int argc, char **argv){
    ros::init(argc, argv, "opti_node");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    RosSystem* sys =  new RosSystem(nh, pnh, 90, "Opti sys");    
    ROSUnit_PoseProvider* providers = new ROSUnit_PoseProvider(nh);
    auto pos_port = sys->createExternalInputPort<Vector3D<float>>("Pos_port");
    auto ori_port = sys->createExternalInputPort<Vector3D<float>>("Ori_port");

    auto opti_port = providers->registerOptiPose("/Robot_1/pose");
    sys->connectExternalInput(pos_port, opti_port[0]);
    sys->connectExternalInput(ori_port, opti_port[1]);

    auto diff_pos = sys->createBlock(BLOCK_ID::DIFFERENTIATOR, "Pos_Derivative", TYPE::Float3); ((Differentiator<Vector3D<float>>*)diff_pos)->supPeak(0.3);
    sys->connectExternalInput(pos_port, diff_pos->getInputPort<Vector3D<float>>(0));
    
    // connect publishers to KF output ports
    sys->createPub<Vector3D<float>>(TYPE::Float3, "/opti/pos", ((Block*)pos_port)->getOutputPort<Vector3D<float>>(0));
    sys->createPub<Vector3D<float>>(TYPE::Float3, "/opti/vel", diff_pos->getOutputPort<Vector3D<float>>(0));
    sys->createPub<Vector3D<float>>(TYPE::Float3, "/opti/ori", ((Block*)ori_port)->getOutputPort<Vector3D<float>>(0));

    sys->start();
    ros::spin();

}