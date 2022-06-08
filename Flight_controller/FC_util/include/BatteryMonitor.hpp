#ifndef BATTERYMONITOR_HPP
#define BATTERYMONITOR_HPP

#include "HEAR_core/Block.hpp"
#include "HEAR_NAVIO_Interface/Navio2/ADC_Navio2.h"
#include <memory>

namespace HEAR {

class BatteryMonitor : public Block {
private:
    OutputPort<float>* bat_volt_port;
    std::unique_ptr <ADC> adc;
    const int CHANNEL = 2;
    const float OFFSET = 0;
    const float SCALE = 1;
public:
    enum OP{BAT_VOLT};
    BatteryMonitor(int b_uid);
    ~BatteryMonitor(){}
    void process();

};

}

#endif