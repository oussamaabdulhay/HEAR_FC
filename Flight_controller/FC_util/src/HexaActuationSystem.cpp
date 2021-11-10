#include "HexaActuationSystem.hpp"

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

HexaActuationSystem::HexaActuationSystem(int b_uid) : Block(BLOCK_ID::HEXAACTUATIONSYSTEM, b_uid){
    body_rate_port = createInputPort<Vector3D<float>>(IP::BODY_RATE_CMD, "BODY_RATE_CMD");
    thrust_port = createInputPort<float>(IP::THRUST_CMD, "THRUST_CMD");
    cmd_out_port = createOutputPort<std::vector<float>>(OP::MOTOR_CMD, "MOTOR_CMD");
}

void HexaActuationSystem::process() {
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

void HexaActuationSystem::heartbeatCb(const std_msgs::Empty::ConstPtr& msg){
    _hb_timer.tick();
}

void HexaActuationSystem::update(UpdateMsg* u_msg){
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

void HexaActuationSystem::setESCValues(int t_armed, int t_min, int t_max) {
    _escMin_armed = t_armed;
    _escMin = t_min;
    _escMax = t_max;
}

void HexaActuationSystem::command(){

    //TODO split into more methods
    for(int i = 0; i < 6; i++){
        _commands[i] = 0.0;
    }

    //Update pulse values
    for(int i = 0; i < 6; i++){
        for(int j = 0; j < 4; j++){
            _commands[i] += _geometry[i][j] * _u[j];
        }
    }

    //_u (PID outputs) should be between 0 and 1. Thus, we have to adjust for the range _escMin_armed to _escMax on _commands.
    //Normalize and Constrain

    for(int i = 0; i < 6; i++){
        _commands[i] = (_commands[i] * (_escMax-_escMin_armed)) + _escMin_armed;
    }

    //Get minimum command
    float min_command = _commands[0];

    for(int i = 1; i < 6; i++){
        if(_commands[i] < min_command){
            min_command = _commands[i];
        }
    }

    float bias = 0;

    //Anti saturation
    if(min_command < _escMin_armed){
        bias = _escMin_armed - min_command;
        
        for(int i = 0; i < 6; i++){
            _commands[i] = _commands[i] + bias;
        }
    }

    for(int i = 0; i < 6; i++){
        if(_armed){
            if(_take_off){
                _commands[i] = this->constrain(_commands[i], _escMin_armed, _escMax);
            }
            else{
                _commands[i] = _escMin_armed;
            }
        }else{
            _commands[i] = _escMin;
        }  
    }

    //Actuate
    for(int i = 0; i < 6; i++){
        _actuators[i]->applyCommand(_commands[i]);
    }

    cmd_out_port->write(_commands);
}

int HexaActuationSystem::constrain(float value, int min_value, int max_value) {
    
    if ((int)value > max_value) {
        value = max_value;
    }
    // bug cathed in the original code
    else if ((int)value < min_value) {
        value = min_value;
    }
    return int(value);
}

}