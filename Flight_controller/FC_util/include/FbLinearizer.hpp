#ifndef FBLINEARIZER_HPP
#define FBLINEARIZER_HPP

#include "HEAR_core/Block.hpp"
#include "HEAR_core/Port.hpp"
#include "HEAR_core/Vector3D.hpp"
#include <tf2/LinearMath/Matrix3x3.h>

#include <iostream>
namespace HEAR{
namespace FbLinearizer{
class Force2Rot : public Block{
private:
    InputPort<float>* yaw_ref_port;
    InputPort<Vector3D<float>>* force_i_des_port;
    OutputPort<tf2::Matrix3x3>* rot_des_port;
    tf2::Matrix3x3 Rot;
public:
    enum IP{YAW_REF, FORCE_I_DES};
    enum OP{ROT_DES};
    Force2Rot(int b_uid);
    ~Force2Rot(){}
    void process();
};

class RotDiff2Rod :  public Block{
private:
    InputPort<tf2::Matrix3x3>* r_i_b_port;
    InputPort<tf2::Matrix3x3>* r_bdes_i_port;
    InputPort<Vector3D<float>>* f_ides_port;
    OutputPort<Vector3D<float>>* angles_port;
    OutputPort<float>* thrust_port;
public:
    enum IP{R_I_B, R_BDES_I, F_IDES};
    enum OP{ROD_ANGLES, THRUST};
    RotDiff2Rod(int b_uid);
    ~RotDiff2Rod(){}
    void process();
};


}

}

#endif