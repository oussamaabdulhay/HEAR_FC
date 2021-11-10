#ifndef ROT2EUL_HPP
#define ROT2EUL_HPP

#include "HEAR_core/Block.hpp"
#include "HEAR_core/Port.hpp"
#include "HEAR_core/Vector3D.hpp"
#include "HEAR_core/DataTypes.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
namespace HEAR{

class Rot2Eul : public Block{
private:
    InputPort<tf2::Matrix3x3>* _inp_port;
    OutputPort<Vector3D<float>>* _out_port;
public:
    enum IP{ROT_MAT};
    enum OP{EUL_ANGLES};
    Rot2Eul(int b_uid);
    ~Rot2Eul(){}
    void process();
};

}

#endif