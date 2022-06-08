
#include "ActuationSysNodelet.h"
#include <pluginlib/class_list_macros.h>

#define HEXA
//#define QUAD

PLUGINLIB_EXPORT_CLASS(HEAR::ActuationSysNodelet, nodelet::Nodelet)

namespace HEAR
{
    ActuationSysNodelet::~ActuationSysNodelet(){
        delete actuation_sys;
    }
    void ActuationSysNodelet::onInit(){
        ros::NodeHandle nh(getNodeHandle());
        ros::NodeHandle pnh(getPrivateNodeHandle());
        actuation_sys = new RosSystem(nh, pnh, FREQUENCY, "ActuationLoop");

        // creating Blocks
    #ifdef QUAD
        auto Uav = new QuadActuationSystem(0);
        actuation_sys->addBlock(Uav, "Quad");
    #else
        auto Uav = new HexaActuationSystem(0);
        actuation_sys->addBlock(Uav, "Hexa");
    #endif

        Uav->init(FREQUENCY);
        Uav->setHbTol(250);
        #ifdef BIG_HEXA
        Uav->setESCValues(1150 ,1000, 1800);
        #else
        Uav->setESCValues(1140 ,1000, 2000);
        #endif

        // thrust to command calculation
        auto act_gain = actuation_sys->createBlock(BLOCK_ID::CONSTANT, "Act_Gain_Kb", TYPE::Float); ((Constant<float>*)act_gain)->setValue(HOV_THRUST/GRAV);
        auto grav_scale = actuation_sys->createBlock(BLOCK_ID::GAIN, "Grav_Normalize"); ((Gain*)grav_scale)->setGain(1.0/GRAV);
        auto hold_thrust_val = actuation_sys->createBlock(BLOCK_ID::HOLDVAL, "Hold_Thrust_Val");
        auto mul_act_gain = actuation_sys->createBlock(BLOCK_ID::MULTIPLY, "Mul_Kb");
        auto mul_adj_gain = actuation_sys->createBlock(BLOCK_ID::MULTIPLY, "Mul_Kadj");
        auto adj_switch = actuation_sys->createBlock(BLOCK_ID::INVERTED_SWITCH, "Adj_Sw");

        auto sub_thrust = actuation_sys->createSub(TYPE::Float, "/thrust_cmd");
        actuation_sys->connect(sub_thrust->getOutputPort<float>(), hold_thrust_val->getInputPort<float>(HoldVal::IP::INPUT));
        actuation_sys->connect(sub_thrust->getOutputPort<float>(), mul_act_gain->getInputPort<float>(Multiply::IP::INPUT_0));
        actuation_sys->connect(hold_thrust_val->getOutputPort<float>(HoldVal::OP::OUTPUT), grav_scale->getInputPort<float>(Gain::IP::INPUT));
        actuation_sys->connect(grav_scale->getOutputPort<float>(Gain::IP::INPUT), mul_adj_gain->getInputPort<float>(Multiply::IP::INPUT_0));
        actuation_sys->connect(act_gain->getOutputPort<float>(Constant<float>::OP::OUTPUT), mul_adj_gain->getInputPort<float>(Multiply::IP::INPUT_1));        
        actuation_sys->connect(act_gain->getOutputPort<float>(Constant<float>::OP::OUTPUT), adj_switch->getInputPort<float>(InvertedSwitch::IP::NC));
        actuation_sys->connect(mul_adj_gain->getOutputPort<float>(Multiply::OP::OUTPUT), adj_switch->getInputPort<float>(InvertedSwitch::IP::NO));
        actuation_sys->connect(adj_switch->getOutputPort<float>(InvertedSwitch::OP::COM), mul_act_gain->getInputPort<float>(Multiply::IP::INPUT_1));
        actuation_sys->connect(mul_act_gain->getOutputPort<float>(Multiply::OP::OUTPUT), Uav->getInputPort<float>(ActuationSystem::IP::THRUST_CMD));

        actuation_sys->createSub(TYPE::Float3, "/angle_u", Uav->getInputPort<Vector3D<float>>(ActuationSystem::IP::BODY_RATE_CMD));
        
        actuation_sys->createPub(TYPE::FloatVec, "/actuation_cmd", Uav->getOutputPort<std::vector<float>>(ActuationSystem::OP::MOTOR_CMD));

        actuation_sys->createPub("/thrust_act", mul_act_gain->getOutputPort<float>(Multiply::OP::OUTPUT));

        actuation_sys->createUpdateTrigger(UPDATE_MSG_TYPE::BOOL_MSG, "/arm", Uav);
        _hb_sub = nh.subscribe("/heartbeat", 10, &ActuationSystem::heartbeatCb, (ActuationSystem*)Uav);
        
        actuation_sys->createUpdateTrigger(UPDATE_MSG_TYPE::BOOL_MSG, "/record_hover_thrust", hold_thrust_val);
        actuation_sys->createUpdateTrigger(UPDATE_MSG_TYPE::BOOL_MSG, "/use_adjusted_act_gain", adj_switch);

        // adding battery monitor
        auto bat_mon = new BatteryMonitor(0);
        actuation_sys->addBlock(bat_mon, "Battery Monitor");
        actuation_sys->createPub("/battery_voltage",bat_mon->getOutputPort<float>(BatteryMonitor::OP::BAT_VOLT));
        //

        actuation_sys->start();

        std::cout << "Created all blocks\n";

    }
    
} // namespace HEAR
