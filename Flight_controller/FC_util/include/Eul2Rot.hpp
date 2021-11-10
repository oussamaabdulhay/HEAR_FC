#ifndef EUL2ROT_HPP
#define EUL2ROT_HPP

#include "HEAR_core/Block.hpp"
#include "HEAR_core/Port.hpp"
#include "HEAR_core/Vector3D.hpp"
#include "HEAR_core/DataTypes.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
namespace HEAR{

class Eul2Rot : public Block{
private:
    InputPort<Vector3D<float>>* _inp_port;
    OutputPort<tf2::Matrix3x3>* _out_port;
    tf2::Matrix3x3 Rot;
public:
    enum IP{EUL_ANGLES};
    enum OP{ROT_MAT};
    Eul2Rot(int b_uid);
    ~Eul2Rot(){}
    void process();
};

}

#endif