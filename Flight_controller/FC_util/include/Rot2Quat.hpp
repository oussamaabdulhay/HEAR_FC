#ifndef ROT2QUAT_HPP
#define ROT2QUAT_HPP

#include "HEAR_core/Block.hpp"
#include "HEAR_core/Port.hpp"
#include "HEAR_core/DataTypes.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
namespace HEAR{

class Rot2Quat : public Block{
private:
    InputPort<tf2::Matrix3x3>* _inp_port;
    OutputPort<tf2::Quaternion>* _out_port;
public:
    enum IP{ROT_MAT};
    enum OP{QUAT};
    Rot2Quat(int b_uid);
    ~Rot2Quat(){}
    void process();
};

}

#endif