#include "Rot2Eul.hpp"

namespace HEAR {

Rot2Eul::Rot2Eul(int b_uid) : Block(BLOCK_ID::ROT2EUL, b_uid){
    _inp_port = createInputPort<tf2::Matrix3x3>(IP::ROT_MAT, "EUL_ANGLES");
    _out_port = createOutputPort<Vector3D<float>>(OP::EUL_ANGLES, "ROT_MAT");

}

void Rot2Eul::process(){
    tf2::Matrix3x3 rot;
    rot.setIdentity();
    _inp_port->read(rot);
    double y, p, r;
    rot.getEulerYPR(y, p, r);
    _out_port->write(Vector3D<float>(r, p, y));
}


}