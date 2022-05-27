#include "ActuationSystem.hpp"

namespace HEAR{

ESCMotor::ESCMotor(int t_pin, int t_freq){
    _pwmPin = t_pin;
    _freq = t_freq;
    this->initialize();
}

bool ESCMotor::initialize(){

    _pwm = new RCOutput_Navio2();

    if(!(_pwm->initialize(_pwmPin)) ) {
        return 1;
    }

    _pwm->set_frequency(_pwmPin, _freq);

	if ( !(_pwm->enable(_pwmPin)) ) {
	    return 1;
	}

}

void ESCMotor::applyCommand(int t_command){

    //std::cout << "Received Command on PIN: " << _pwmPin << " Value :" << t_command << "\r"; //std::endl;
    _pwm->set_duty_cycle(_pwmPin, t_command);

}

ActuationSystem::ActuationSystem(BLOCK_ID block_id, int b_uid) : Block(block_id, b_uid){
    body_rate_port = createInputPort<Vector3D<float>>(IP::BODY_RATE_CMD, "BODY_RATE_CMD");
    thrust_port = createInputPort<float>(IP::THRUST_CMD, "THRUST_CMD");
    cmd_out_port = createOutputPort<std::vector<float>>(OP::MOTOR_CMD, "MOTOR_CMD");
}


void ActuationSystem::heartbeatCb(const std_msgs::Empty::ConstPtr& msg){
    _hb_timer.tick();
}

void ActuationSystem::update(UpdateMsg* u_msg){
    if(u_msg->getType() == UPDATE_MSG_TYPE::BOOL_MSG){
        bool arm_val = ((BoolMsg*)u_msg)->data;;
        if(arm_val){
            if(_armed){
                _take_off = arm_val;
                std::cout << "take off \n";
            }
            else{
                _armed = arm_val;
                std::cout << "arm called \n";
            }
        }
        else{
            _armed = arm_val;
            _take_off = arm_val;
            std::cout << "disarm called \n";
        }
        // print armed
    }
}

void ActuationSystem::process() {
    if ( _hb_timer.tockMilliSeconds() > _hb_tol_ms){
        _armed = false;
        _take_off = false;
    }
    Vector3D<float> body_rate_cmd;
    body_rate_port->read(body_rate_cmd);
    _u[0] = body_rate_cmd.x; 
    _u[1] = body_rate_cmd.y;
    _u[2] = body_rate_cmd.z;
    thrust_port->read(_u[3]);
    this->command();
}

void ActuationSystem::setESCValues(int t_armed, int t_min, int t_max) {
    _escMin_armed = t_armed;
    _escMin = t_min;
    _escMax = t_max;
}

int ActuationSystem::constrain(float value, int min_value, int max_value) {
    int val = static_cast<int>(value);
    if (val > max_value) {
        val = max_value;
    }
    // bug cathed in the original code
    else if (val < min_value) {
        val = min_value;
    }
    return val;
}

}