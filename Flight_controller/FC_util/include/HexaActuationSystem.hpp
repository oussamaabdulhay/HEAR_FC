#ifndef HEXAACTUATIONSYSTEM_HPP
#define HEXAACTUATIONSYSTEM_HPP

#include <unistd.h>
#include <memory>
#include "HEAR_NAVIO_Interface/Navio2/RCOutput_Navio2.h"

#include "HEAR_core/Block.hpp"
#include "HEAR_core/Port.hpp"
#include "HEAR_core/Vector3D.hpp"
#include "HEAR_core/Timer.hpp"
#include "std_msgs/Empty.h"

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

class  Actuator {
    public:
        virtual bool initialize() = 0;
        virtual void applyCommand(int command) = 0;
        Actuator() {};
};

class ESCMotor : public Actuator {
private:
    int _pwmPin;
    int _freq;
    RCOutput* _pwm;
public:
    bool initialize();
    void applyCommand(int command);
    RCOutput* get_rcout();
    ESCMotor(int, int);
};

class HexaActuationSystem : public Block {
private: 
    Timer _hb_timer;
    int _hb_tol_ms = 1000;
    std::vector<Actuator*> _actuators;
    int _escMin = 1000;
    int _escMin_armed = 1150;
    int _escMax = 2000;
    bool _armed = false;
    bool _take_off = false;
    float _u[4]; //[roll, pitch, yaw, throttle]
    std::vector<float> _commands {0,0,0,0,0,0};
    float _geometry[6][4] = {{-1,    0,          1,     1},
                             { 1,    0,         -1,     1},
                             { 0.5, -0.866025,   1,     1},
                             {-0.5,  0.866025,  -1,     1},
                             {-0.5, -0.866025,  -1,     1},
                             { 0.5,  0.866025,   1,     1}};
    InputPort<Vector3D<float>>* body_rate_port;
    InputPort<float>* thrust_port;
    OutputPort<std::vector<float>>* cmd_out_port;
    void command();
    int constrain(float value, int min_value, int max_value);

public:
    enum IP{BODY_RATE_CMD, THRUST_CMD};
    enum OP{MOTOR_CMD};
    void process();
    void heartbeatCb(const std_msgs::Empty::ConstPtr& msg);
    void setHbTol(int hb_tol_ms){
        _hb_tol_ms = hb_tol_ms;
    }
    void update(UpdateMsg* u_msg) override;
    void setESCValues(int, int, int);
    void init(const int& pwm_freq){
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