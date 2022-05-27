#include "QuadActuationSystem.hpp"

namespace HEAR{


QuadActuationSystem::QuadActuationSystem(int b_uid) : ActuationSystem(BLOCK_ID::QUADACTUATIONSYSTEM, b_uid){
}

void QuadActuationSystem::command(){


    for(int i = 0; i < NUM_MOTORS; i++){
        _commands[i] = 0.0;
    }

    //Update pulse values
    for(int i = 0; i < NUM_MOTORS; i++){
        for(int j = 0; j < 4; j++){
            _commands[i] += _geometry[i][j] * _u[j];
        }
    }

    //_u (PID outputs) should be between 0 and 1. Thus, we have to adjust for the range _escMin_armed to _escMax on _commands.
    //Normalize and Constrain

    for(int i = 0; i < NUM_MOTORS; i++){
        _commands[i] = (_commands[i] * (_escMax-_escMin_armed)) + _escMin_armed;
    }

    //Get minimum command
    float min_command = _commands[0];

    for(int i = 1; i < NUM_MOTORS; i++){
        if(_commands[i] < min_command){
            min_command = _commands[i];
        }
    }

    float bias = 0;

    //Anti saturation
    if(min_command < _escMin_armed){
        bias = _escMin_armed - min_command;
        for(int i = 0; i < NUM_MOTORS; i++){
            _commands[i] = _commands[i] + bias;
        }
    }

    for(int i = 0; i < NUM_MOTORS; i++){
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
    for(int i = 0; i < NUM_MOTORS; i++){
        _actuators[i]->applyCommand(_commands[i]);
    }

    cmd_out_port->write(_commands);
}

}