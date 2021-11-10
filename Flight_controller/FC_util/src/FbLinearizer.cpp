#include "FbLinearizer.hpp"

namespace HEAR{

namespace FbLinearizer{

Force2Rot::Force2Rot(int b_uid) : Block(BLOCK_ID::FORCE2ROT, b_uid){
    yaw_ref_port = createInputPort<float>(IP::YAW_REF, "YAW_REF");
    force_i_des_port = createInputPort<Vector3D<float>>(IP::FORCE_I_DES, "FORCE_I_DES");
    rot_des_port = createOutputPort<tf2::Matrix3x3>(OP::ROT_DES, "ROT_DES");

}

void Force2Rot::process(){
    Vector3D<float> F_Ides(0, 0, 0);
    float yaw_ref = 0;
    yaw_ref_port->read(yaw_ref);
    force_i_des_port->read(F_Ides);
    // std::cout << "F_I_des : " <<F_I_des.x << " " << F_I_des.y << " " << F_I_des.z << std::endl;
    // std::cout << "yaw_ref : " << yaw_ref << std::endl;
    if (F_Ides.z < 0.1){
        F_Ides.z = 0.1;
    }
    tf2::Vector3 F_I_des(F_Ides.x, F_Ides.y, F_Ides.z);
    auto z_B_des = F_I_des.normalized();
    auto x_aux = (tf2::Vector3(-sin(yaw_ref), cos(yaw_ref), 0.0)).cross(z_B_des);
    auto x_B_des = x_aux.normalized();
    auto y_B_des = z_B_des.cross(x_B_des); //TODO: check if it is z_B or z_B_des
    auto R_B_des_I = tf2::Matrix3x3(x_B_des.x(), x_B_des.y(), x_B_des.z(),
                                    y_B_des.x(), y_B_des.y(), y_B_des.z(),
                                    z_B_des.x(), z_B_des.y(), z_B_des.z());
    rot_des_port->write(R_B_des_I);                                
}

RotDiff2Rod::RotDiff2Rod(int b_uid) : Block(BLOCK_ID::ROTDIFF2ROD, b_uid){
    r_i_b_port = createInputPort<tf2::Matrix3x3>(IP::R_I_B, "R_I_B");
    f_ides_port = createInputPort<Vector3D<float>>(IP::F_IDES, "F_IDES");
    r_bdes_i_port = createInputPort<tf2::Matrix3x3>(IP::R_BDES_I, "R_BDES_I");
    angles_port = createOutputPort<Vector3D<float>>(OP::ROD_ANGLES, "ROD_ANGLES");
    thrust_port = createOutputPort<float>(OP::THRUST, "THRUST");

}

void RotDiff2Rod::process(){
    tf2::Matrix3x3 R_I_B, R_B_des_I;
    R_I_B.setIdentity(); R_B_des_I.setIdentity();
    Vector3D<float> F_I_des(0, 0, 0);
    r_i_b_port->read(R_I_B);
    r_bdes_i_port->read(R_B_des_I);
    f_ides_port->read(F_I_des);
    
    auto R_B_B_des = R_I_B.transposeTimes(R_B_des_I.transpose());
    tf2::Quaternion quat;
    R_B_B_des.getRotation(quat);
    auto angle = quat.getAngle();
    auto err_angles = (angle <= M_PI? angle: angle - 2*M_PI)*quat.getAxis();

    auto u_z = (R_I_B.transpose()*tf2::Vector3(F_I_des.x, F_I_des.y, F_I_des.z)).z();

    angles_port->write(Vector3D<float>((float)err_angles.x(), (float)err_angles.y(), (float)err_angles.z()));
    thrust_port->write(u_z);
}
}
}