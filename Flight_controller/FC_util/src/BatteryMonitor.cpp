#include "BatteryMonitor.hpp"

namespace HEAR{

BatteryMonitor::BatteryMonitor(int b_uid) : Block(BLOCK_ID::BATTERYMONITOR, b_uid){
    bat_volt_port = createOutputPort<float>(OP::BAT_VOLT, "BAT_VOLT");
    adc = std::unique_ptr <ADC>{ new ADC_Navio2() };
    adc->initialize();
}

void BatteryMonitor::process(){
    float reading = adc->read(CHANNEL);
    bat_volt_port->write(reading*SCALE+OFFSET);
}

}