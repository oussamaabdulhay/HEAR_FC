#ifndef QUAT2ROT_HPP
#define QUAT2ROT_HPP

#include "HEAR_core/Block.hpp"
#include "HEAR_core/Port.hpp"
#include "HEAR_core/DataTypes.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
namespace HEAR{

class Quat2Rot : public Block{
private:
    InputPort<tf2::Quaternion>* _inp_port;
    OutputPort<tf2::Matrix3x3>* _out_port;
    tf2::Matrix3x3 Rot;
public:
    enum IP{QUAT};
    enum OP{ROT_MAT};
    Quat2Rot(int b_uid);
    ~Quat2Rot(){}
    void process();
};

}

#endif