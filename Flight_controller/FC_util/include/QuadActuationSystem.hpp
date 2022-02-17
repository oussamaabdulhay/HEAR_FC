#ifndef QUADACTUATIONSYSTEM_HPP
#define QUADACTUATIONSYSTEM_HPP

#include "ActuationSystem.hpp"

// GEOMETRY
//      CW(1) (3)CCW                x
//          \ /                     ↑
//           X                      |
//          / \              y <----+ 
//     CCW(4) (2)CW               z up
//
// For Positive Pitch, all motors with negative X should be increased
// For Negative Roll, all motors with negative Y should be increased
// For Positive Yaw, all motors with CW should be increased
// Mx = [x, y, direction, thottle]
// POSITIVE PITCH result in moving in the direction of POSITIVE X
// NEGATIVE ROLL result in moving in the direction of POSITIVE Y

namespace HEAR{


class QuadActuationSystem : public ActuationSystem {

private:    
    const int NUM_MOTORS = 4;

    std::vector<float> _commands {0,0,0,0};
    float _geometry[4][4] = {{-0.707107, -0.707107,  1, 1},
                             { 0.707107,  0.707107,  1, 1},
                             { 0.707107, -0.707107, -1, 1},
                             {-0.707107,  0.707107, -1, 1}};
    void command();

public:
    void init(const int pwm_freq){
        auto M1 = new ESCMotor(0, pwm_freq);
        auto M2 = new ESCMotor(1, pwm_freq);
        auto M3 = new ESCMotor(2, pwm_freq);
        auto M4 = new ESCMotor(3, pwm_freq);
        _actuators={M1, M2, M3, M4};
    }
    QuadActuationSystem(int);
    ~QuadActuationSystem(){}
};

}

#endif