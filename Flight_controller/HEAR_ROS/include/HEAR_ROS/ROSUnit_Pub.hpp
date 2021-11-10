#ifndef ROSUNIT_PUB_HPP
#define ROSUNIT_PUB_HPP

#include <ros/ros.h>
#include "HEAR_core/DataTypes.hpp"
#include "HEAR_core/Port.hpp"

namespace HEAR{

class ROSUnit_Pub {
protected:
    int id_; 
    ros::Publisher pub_;
    Port* _input_port;
public:
    template <class T> InputPort<T>* getInputPort() { return (InputPort<T>*)_input_port;}

    virtual TYPE getType() = 0;
    virtual void process() = 0;
    virtual int getID() const {return id_;}
};
}

#endif