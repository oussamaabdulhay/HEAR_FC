#ifndef ROSUNIT_SUB_HPP
#define ROSUNIT_SUB_HPP

#include <ros/ros.h>
#include "HEAR_core/DataTypes.hpp"
#include "HEAR_core/Port.hpp"

namespace HEAR{

class ROSUnit_Sub {
protected:
    int id_; 
    ros::Subscriber sub_;
    Port* _output_port;
public:
    template <class T> OutputPort<T>* getOutputPort() { return (OutputPort<T>*)_output_port;}

    virtual TYPE getType() = 0;
    virtual int getID() const {return id_;}
};
}

#endif
