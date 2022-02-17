#ifndef ACTUATIONSYSTEM_HPP
#define ACTUATIONSYSTEM_HPP

#include <unistd.h>
#include <memory>
#include "HEAR_NAVIO_Interface/Navio2/RCOutput_Navio2.h"

#include "HEAR_core/Block.hpp"
#include "HEAR_core/Port.hpp"
#include "HEAR_core/Vector3D.hpp"
#include "HEAR_core/Timer.hpp"
#include "std_msgs/Empty.h"

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

class ActuationSystem : public Block {
protected: 
    Timer _hb_timer;
    int _hb_tol_ms = 1000;
    std::vector<Actuator*> _actuators;
    int _escMin = 1000;
    int _escMin_armed = 1150;
    int _escMax = 2000;
    bool _armed = false;
    bool _take_off = false;
    float _u[4]; //[roll, pitch, yaw, throttle]

    InputPort<Vector3D<float>>* body_rate_port;
    InputPort<float>* thrust_port;
    OutputPort<std::vector<float>>* cmd_out_port;
    int constrain(float value, int min_value, int max_value);
    virtual void command() = 0;

public:
    enum IP{BODY_RATE_CMD, THRUST_CMD};
    enum OP{MOTOR_CMD};
    void process();
    virtual void init(const int) = 0;
    void heartbeatCb(const std_msgs::Empty::ConstPtr& msg);
    void setHbTol(int hb_tol_ms){
        _hb_tol_ms = hb_tol_ms;
    }
    void update(UpdateMsg* u_msg) override;
    void setESCValues(int, int, int);
    ActuationSystem(BLOCK_ID, int);
    ~ActuationSystem(){}
}; 

}

#endif