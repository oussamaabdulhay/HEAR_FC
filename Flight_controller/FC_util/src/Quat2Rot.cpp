#include "Quat2Rot.hpp"

namespace HEAR {

Quat2Rot::Quat2Rot(int b_uid) : Block(BLOCK_ID::QUAT2ROT, b_uid){
    _inp_port = createInputPort<tf2::Quaternion>(IP::QUAT, "QUAT_ANGLES");
    _out_port = createOutputPort<tf2::Matrix3x3>(OP::ROT_MAT, "ROT_MAT");

}

void Quat2Rot::process(){
    tf2::Quaternion quat(0, 0, 0, 1);
    _inp_port->read(quat);
    Rot.setRotation(quat);
    _out_port->write(Rot);
}


}