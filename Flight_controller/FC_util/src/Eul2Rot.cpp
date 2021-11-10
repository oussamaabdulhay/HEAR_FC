#include "Eul2Rot.hpp"

namespace HEAR {

Eul2Rot::Eul2Rot(int b_uid) : Block(BLOCK_ID::EUL2ROT, b_uid){
    _inp_port = createInputPort<Vector3D<float>>(IP::EUL_ANGLES, "EUL_ANGLES");
    _out_port = createOutputPort<tf2::Matrix3x3>(OP::ROT_MAT, "ROT_MAT");

}

void Eul2Rot::process(){
    Vector3D<float> eul(0, 0, 0);
    _inp_port->read(eul);
    Rot.setEulerYPR(eul.z, eul.y, eul.x);
    _out_port->write(Rot);
}


}