#ifndef HEXAACTUATIONSYSTEM_HPP
#define HEXAACTUATIONSYSTEM_HPP

#include "ActuationSystem.hpp"

// Using ENU Reference Frame
// GEOMETRY
//      CW(3) (5)CCW                x
//          \ /                     ↑
// CCW(2) -- X -- (1)CW             |
//          / \              y <----+ 
//      CW(6) (4)CCW               z up
//
// For Positive Pitch, all motors with negative X should be increased
// For Negative Roll, all motors with negative Y should be increased
// For Positive Yaw, all motors with CW should be increased
// Mx = [x, y, direction, thottle]
// POSITIVE PITCH result in moving in the direction of POSITIVE X
// NEGATIVE ROLL result in moving in the direction of POSITIVE Y
// Using YPR rotation sequence in the construction of the Rotation Matrix

namespace HEAR{

class HexaActuationSystem : public ActuationSystem {
private: 
    std::vector<float> _commands {0,0,0,0,0,0};
    float _geometry[6][4] = {{-1,    0,          1,     1},
                             { 1,    0,         -1,     1},
                             { 0.5, -0.866025,   1,     1},
                             {-0.5,  0.866025,  -1,     1},
                             {-0.5, -0.866025,  -1,     1},
                             { 0.5,  0.866025,   1,     1}};
    void command();

public:
    void init(const int pwm_freq){
        auto M1 = new ESCMotor(0, pwm_freq);
        auto M2 = new ESCMotor(1, pwm_freq);
        auto M3 = new ESCMotor(2, pwm_freq);
        auto M4 = new ESCMotor(3, pwm_freq);
        auto M5 = new ESCMotor(4, pwm_freq);
        auto M6 = new ESCMotor(5, pwm_freq);
        _actuators={M1, M2, M3, M4, M5, M6};
    }
    HexaActuationSystem(int);
    ~HexaActuationSystem(){}
}; 

}

#endif