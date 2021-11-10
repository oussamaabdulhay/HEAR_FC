#ifndef FROMHORIZON_HPP
#define FROMHORIZON_HPP

#include "HEAR_core/Block.hpp"
#include "HEAR_core/Port.hpp"
#include "HEAR_core/Vector3D.hpp"
#include "HEAR_core/DataTypes.hpp"

#include <tf2/LinearMath/Matrix3x3.h>

namespace HEAR{
class FromHorizon : public Block{
private:
    InputPort<float>* yaw_port;
    InputPort<Vector3D<float>>* inp_vec_port;
    OutputPort<Vector3D<float>>* out_vec_port;
    tf2::Matrix3x3 Rot;
public:
    enum IP{YAW, INP_VEC};
    enum OP{OUT_VEC};
    FromHorizon(int b_uid);
    ~FromHorizon(){}
    void process();

};

}
#endif