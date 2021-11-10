#include "Rot2Quat.hpp"

namespace HEAR {

Rot2Quat::Rot2Quat(int b_uid) : Block(BLOCK_ID::ROT2QUAT, b_uid){
    _inp_port = createInputPort<tf2::Matrix3x3>(IP::ROT_MAT, "EUL_ANGLES");
    _out_port = createOutputPort<tf2::Quaternion>(OP::QUAT, "ROT_MAT");

}

void Rot2Quat::process(){
    tf2::Matrix3x3 rot;
    rot.setIdentity();
    _inp_port->read(rot);
    tf2::Quaternion quat;
    rot.getRotation(quat);
    _out_port->write(quat);
}


}