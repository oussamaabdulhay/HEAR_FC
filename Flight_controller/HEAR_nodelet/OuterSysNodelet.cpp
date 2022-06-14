
#include "OuterSysNodelet.h"
#include <pluginlib/class_list_macros.h>



PLUGINLIB_EXPORT_CLASS(HEAR::OuterSysNodelet, nodelet::Nodelet)

namespace HEAR
{
    OuterSysNodelet::~OuterSysNodelet(){
        delete outer_sys;
    }
    void OuterSysNodelet::onInit(){
        ros::NodeHandle nh(getNodeHandle());
        ros::NodeHandle pnh(getPrivateNodeHandle());
        outer_sys = new RosSystem(nh, pnh, FREQUENCY, "OuterLoop");

        //////////// creating blocks /////////////
        auto pos_filt = outer_sys->createBlock(BLOCK_ID::BW_FILT2, "Vel_Filt", TYPE::Float3);
        ((BWFilter2<Vector3D<float>>*)pos_filt)->setCoeff(BWFilt2_coeff::coeff_120Hz_2nd_butter_5hz);
        auto demux_ori = outer_sys->createBlock(BLOCK_ID::DEMUX3, "Demux_ori");
        auto to_horizon_pos = new ToHorizon(0);
        outer_sys->addBlock(to_horizon_pos, "ToHorizon_Pos");
        auto to_horizon_vel = new ToHorizon(0);
        outer_sys->addBlock(to_horizon_vel, "ToHorizon_Vel");
        auto to_horizon_acc = new ToHorizon(0);
        outer_sys->addBlock(to_horizon_acc, "ToHorizon_Acc");

        auto pos_kf_demux = outer_sys->createBlock(BLOCK_ID::DEMUX3, "Pos_KF_Demux");
        auto pos_opti_demux = outer_sys->createBlock(BLOCK_ID::DEMUX3, "Pos_OPTI_Demux");
        auto vel_kf_demux = outer_sys->createBlock(BLOCK_ID::DEMUX3, "Vel_KF_Demux");
        auto vel_opti_demux = outer_sys->createBlock(BLOCK_ID::DEMUX3, "Vel_OPTI_Demux");

        auto pos_x_sw = outer_sys->createBlock(BLOCK_ID::INVERTED_SWITCH3, "Pos_sw");
        auto pos_z_sw = outer_sys->createBlock(BLOCK_ID::INVERTED_SWITCH3, "Pos_sw");

        auto vel_x_sw = outer_sys->createBlock(BLOCK_ID::INVERTED_SWITCH3, "Vel_sw");
        auto vel_z_sw = outer_sys->createBlock(BLOCK_ID::INVERTED_SWITCH3, "Vel_sw");




        auto mux_pos = outer_sys->createBlock(BLOCK_ID::MUX3, "Mux_POS");

        auto mux_vel = outer_sys->createBlock(BLOCK_ID::MUX3, "Mux_VEL");


        auto pos_h_demux = outer_sys->createBlock(BLOCK_ID::DEMUX3, "Pos_H_Demux");
        auto vel_h_demux = outer_sys->createBlock(BLOCK_ID::DEMUX3, "Vel_H_Demux");
        auto acc_h_demux = outer_sys->createBlock(BLOCK_ID::DEMUX3, "Acc_H_Demux");
        auto sum_pos_err = outer_sys->createBlock(BLOCK_ID::SUM3, "Sum_Pos_Err");
        auto sum_vel_err = outer_sys->createBlock(BLOCK_ID::SUM3, "Sum_Vel_Err");
        auto pid_x = outer_sys->createBlock(BLOCK_ID::PID, "Pid_x"); ((PID_Block*)pid_x)->setPID_ID(PID_ID::PID_X);
        auto pid_y = outer_sys->createBlock(BLOCK_ID::PID, "Pid_y"); ((PID_Block*)pid_y)->setPID_ID(PID_ID::PID_Y);
        auto pid_z = outer_sys->createBlock(BLOCK_ID::PID, "Pid_z"); ((PID_Block*)pid_z)->setPID_ID(PID_ID::PID_Z);
        auto sat_x = outer_sys->createBlock(BLOCK_ID::SATURATION, "Sat_x"); ((Saturation*)sat_x)->setClipValue(SAT_XY_VALUE); 
        auto sat_y = outer_sys->createBlock(BLOCK_ID::SATURATION, "Sat_y"); ((Saturation*)sat_y)->setClipValue(SAT_XY_VALUE);
        auto sat_z = outer_sys->createBlock(BLOCK_ID::SATURATION, "Sat_z"); ((Saturation*)sat_z)->setClipValue(SAT_Z_VALUE);
        auto mux_fh_des = outer_sys->createBlock(BLOCK_ID::MUX3, "Mux_Fh");
        auto fh2fi = new FromHorizon(0);
        outer_sys->addBlock(fh2fi, "Fh_to_Fi");
        auto f2rot = new FbLinearizer::Force2Rot(0);
        outer_sys->addBlock(f2rot, "Fi_to_RotMat");
        auto rot2eul = new Rot2Eul(0);
        outer_sys->addBlock(rot2eul, "RotMat_to_Eul");
        auto demux_eul_des = outer_sys->createBlock(BLOCK_ID::DEMUX3, "Demux_EulDes");
        auto mux_eul_des = outer_sys->createBlock(BLOCK_ID::MUX3, "Mux_EulDes");
        auto bias_z = outer_sys->createBlock(BLOCK_ID::CONSTANT, "Bias_z", TYPE::Float); ((Constant<float>*)bias_z)->setValue(BIAS_Z_VALUE);
        auto sum_bias_z = outer_sys->createBlock(BLOCK_ID::SUM, "Sum_bias_z"); ((Sum*)sum_bias_z)->setOperation(Sum::OPERATION::ADD);

        // bias correction blocks
        auto hold_x_bias = outer_sys->createBlock(BLOCK_ID::HOLDVAL, "Hold_X_Cmd");
        auto hold_y_bias = outer_sys->createBlock(BLOCK_ID::HOLDVAL, "Hold_Y_Cmd");
        auto sw_bias_x = outer_sys->createBlock(BLOCK_ID::INVERTED_SWITCH, "Sw_Bias_X");
        auto sw_bias_y = outer_sys->createBlock(BLOCK_ID::INVERTED_SWITCH, "Sw_Bias_Y");
        auto sum_cmd_bias_x = outer_sys->createBlock(BLOCK_ID::SUM, "Sum_Cmd_Bias_x"); ((Sum*)sum_cmd_bias_x)->setOperation(Sum::OPERATION::ADD);
        auto sum_cmd_bias_y = outer_sys->createBlock(BLOCK_ID::SUM, "Sum_Cmd_Bias_y"); ((Sum*)sum_cmd_bias_y)->setOperation(Sum::OPERATION::ADD);
                
        // feedforward acceleration blocks
        auto sum_acc_x = outer_sys->createBlock(BLOCK_ID::SUM, "Sum_acc_x"); ((Sum*)sum_acc_x)->setOperation(Sum::OPERATION::ADD);
        auto sum_acc_y = outer_sys->createBlock(BLOCK_ID::SUM, "Sum_acc_y"); ((Sum*)sum_acc_y)->setOperation(Sum::OPERATION::ADD);
        // auto grav_scale = outer_sys->createBlock(BLOCK_ID::GAIN, "Grav_Normalize"); ((Gain*)grav_scale)->setGain(1.0/9.8);
        // auto hold_thrust_val = outer_sys->createBlock(BLOCK_ID::HOLDVAL, "Hold_Thrust_Val");
        // auto acc_ref_gain_x = outer_sys->createBlock(BLOCK_ID::MULTIPLY, "Acc_Ref_Gain_x");
        // auto acc_ref_gain_y = outer_sys->createBlock(BLOCK_ID::MULTIPLY, "Acc_Ref_Gain_y");

        //////// creating MRFT specific blocks ////////////
        auto mrft_x = outer_sys->createBlock(BLOCK_ID::MRFT, "Mrft_x"); ((MRFT_Block*)mrft_x)->setMRFT_ID(MRFT_ID::MRFT_X);
        auto mrft_y = outer_sys->createBlock(BLOCK_ID::MRFT, "Mrft_y"); ((MRFT_Block*)mrft_y)->setMRFT_ID(MRFT_ID::MRFT_Y);
        auto mrft_z = outer_sys->createBlock(BLOCK_ID::MRFT, "Mrft_z"); ((MRFT_Block*)mrft_z)->setMRFT_ID(MRFT_ID::MRFT_Z);
        auto mrft_sw_x = outer_sys->createBlock(BLOCK_ID::INVERTED_SWITCH, "Mrft_Sw_x");
        auto mrft_sw_y = outer_sys->createBlock(BLOCK_ID::INVERTED_SWITCH, "Mrft_Sw_y");
        auto mrft_sw_z = outer_sys->createBlock(BLOCK_ID::INVERTED_SWITCH, "Mrft_Sw_z");
        auto med_filt_x = outer_sys->createBlock(BLOCK_ID::MEDIAN_FILTER, "Med_Filt_x"); ((MedianFilter*)med_filt_x)->setWinSize(50);
        auto med_filt_y = outer_sys->createBlock(BLOCK_ID::MEDIAN_FILTER, "Med_Filt_y"); ((MedianFilter*)med_filt_y)->setWinSize(50);
        auto med_filt_z = outer_sys->createBlock(BLOCK_ID::MEDIAN_FILTER, "Med_Filt_z"); ((MedianFilter*)med_filt_z)->setWinSize(50);

        //////// creating SLAM specific Blocks ///////////
        // auto diff_slam_pos = outer_sys->createBlock(BLOCK_ID::DIFFERENTIATOR, "Pos_Derivative", TYPE::Float3); ((Differentiator<Vector3D<float>>*)diff_slam_pos)->supPeak(1.0);
                
        ////////////// connecting blocks /////////

        // external input for slam
        outer_sys->createSub(TYPE::Float3 ,"/kf/position", pos_kf_demux->getInputPort<Vector3D<float>>(Demux3::IP::INPUT));
        outer_sys->createSub(TYPE::Float3, "/kf/velocity", vel_kf_demux->getInputPort<Vector3D<float>>(Demux3::IP::INPUT));


        // connecting input data preparation blocks
        outer_sys->createSub(TYPE::Float3, "opti/pos", pos_opti_demux->getInputPort<Vector3D<float>>(Demux3::IP::INPUT));
        outer_sys->createSub(TYPE::Float3, "/opti/vel", vel_opti_demux->getInputPort<Vector3D<float>>(Demux3::IP::INPUT));

        outer_sys->connect(pos_kf_demux->getOutputPort<Vector3D<float>>(Demux3::OP::X), pos_x_sw->getInputPort<Vector3D<float>>(InvertedSwitch3::OP::NO));
        outer_sys->connect(pos_opti_demux->getOutputPort<Vector3D<float>>(Demux3::OP::X), pos_x_sw->getInputPort<Vector3D<float>>(InvertedSwitch3::OP::NC));

        outer_sys->connect(pos_kf_demux->getOutputPort<Vector3D<float>>(Demux3::OP::Z), pos_z_sw->getInputPort<Vector3D<float>>(InvertedSwitch3::OP::NO));
        outer_sys->connect(pos_opti_demux->getOutputPort<Vector3D<float>>(Demux3::OP::Z), pos_z_sw->getInputPort<Vector3D<float>>(InvertedSwitch3::OP::NC));

        outer_sys->connect(vel_kf_demux->getOutputPort<Vector3D<float>>(Demux3::OP::X), pos_x_sw->getInputPort<Vector3D<float>>(InvertedSwitch3::OP::NO));
        outer_sys->connect(vel_opti_demux->getOutputPort<Vector3D<float>>(Demux3::OP::X), pos_x_sw->getInputPort<Vector3D<float>>(InvertedSwitch3::OP::NC));

        outer_sys->connect(vel_kf_demux->getOutputPort<Vector3D<float>>(Demux3::OP::Z), vel_x_sw->getInputPort<Vector3D<float>>(InvertedSwitch3::OP::NO));
        outer_sys->connect(vel_opti_demux->getOutputPort<Vector3D<float>>(Demux3::OP::Z), vel_z_sw->getInputPort<Vector3D<float>>(InvertedSwitch3::OP::NC));

        outer_sys->connect(pos_x_sw->getOutputPort<Vector3D<float>>(InvertedSwitch3::OP::COM), mux_pos->getInputPort<float>(Mux3::IP::X));
        outer_sys->connect(pos_opti_demux->getOutputPort<Vector3D<float>>(Demux3::OP::Y, mux_pos->getInputPort<float>(Mux3::IP::Y));
        outer_sys->connect(pos_z_sw->getOutputPort<Vector3D<float>>(InvertedSwitch3::OP::COM), mux_pos->getInputPort<float>(Mux3::IP::Z));

        outer_sys->connect(vel_x_sw->getOutputPort<Vector3D<float>>(InvertedSwitch3::OP::COM), mux_vel->getInputPort<float>(Mux3::IP::X));
        outer_sys->connect(vel_opti_demux->getOutputPort<Vector3D<float>>(Demux3::OP::Y, mux_vel->getInputPort<float>(Mux3::IP::Y));
        outer_sys->connect(vel_z_sw->getOutputPort<Vector3D<float>>(InvertedSwitch3::OP::COM), mux_vel->getInputPort<float>(Mux3::IP::Z));

        outer_sys->connect(mux_vel->getOutputPort<Vector3D<float>>(Mux3::OP::OUTPUT), pos_filt->getInputPort<Vector3D<float>>(0));
        outer_sys->createSub(TYPE::Float3, "opti/ori", demux_ori->getInputPort<Vector3D<float>>(Demux3::IP::INPUT));
       

        outer_sys->createSub(TYPE::Float3, "/waypoint_reference/pos", sum_pos_err->getInputPort<Vector3D<float>>(Sum3::IP::OPERAND1));
        outer_sys->createSub(TYPE::Float3, "/waypoint_reference/vel", sum_vel_err->getInputPort<Vector3D<float>>(Sum3::IP::OPERAND1));
        outer_sys->connect(mux_pos->getOutputPort<Vector3D<float>>(Mux3::OP::OUTPUT), sum_pos_err->getInputPort<Vector3D<float>>(Sum3::IP::OPERAND2));
        outer_sys->connect(pos_filt->getOutputPort<Vector3D<float>>(0), sum_vel_err->getInputPort<Vector3D<float>>(Sum3::IP::OPERAND2));
        outer_sys->connect(sum_pos_err->getOutputPort<Vector3D<float>>(Sum3::OP::OUTPUT), to_horizon_pos->getInputPort<Vector3D<float>>(ToHorizon::IP::INP_VEC));
        outer_sys->connect(sum_vel_err->getOutputPort<Vector3D<float>>(Sum3::OP::OUTPUT), to_horizon_vel->getInputPort<Vector3D<float>>(ToHorizon::IP::INP_VEC));
        outer_sys->connect(demux_ori->getOutputPort<float>(Demux3::OP::Z), to_horizon_pos->getInputPort<float>(ToHorizon::IP::YAW));
        outer_sys->connect(demux_ori->getOutputPort<float>(Demux3::OP::Z), to_horizon_vel->getInputPort<float>(ToHorizon::IP::YAW));
        outer_sys->connect(to_horizon_pos->getOutputPort<Vector3D<float>>(ToHorizon::OUT_VEC), pos_h_demux->getInputPort<Vector3D<float>>(Demux3::INPUT));
        outer_sys->connect(to_horizon_vel->getOutputPort<Vector3D<float>>(ToHorizon::OUT_VEC), vel_h_demux->getInputPort<Vector3D<float>>(Demux3::INPUT));

        outer_sys->createSub(TYPE::Float3, "/waypoint_reference/acc", to_horizon_acc->getInputPort<Vector3D<float>>(ToHorizon::IP::INP_VEC));
        outer_sys->connect(demux_ori->getOutputPort<float>(Demux3::OP::Z), to_horizon_acc->getInputPort<float>(ToHorizon::IP::YAW));
        outer_sys->connect(to_horizon_acc->getOutputPort<Vector3D<float>>(ToHorizon::OUT_VEC), acc_h_demux->getInputPort<Vector3D<float>>(Demux3::INPUT));

        // connecting x control sys blocks
        // outer_sys->createSub("/thrust_cmd", hold_thrust_val->getInputPort<float>(HoldVal::IP::INPUT));
        // outer_sys->connect(hold_thrust_val->getOutputPort<float>(HoldVal::OP::OUTPUT), grav_scale->getInputPort<float>(Gain::IP::INPUT));
        // outer_sys->connect(grav_scale->getOutputPort<float>(Gain::OP::OUTPUT), acc_ref_gain_x->getInputPort<float>(Multiply::IP::INPUT_0));
        // outer_sys->connect(grav_scale->getOutputPort<float>(Gain::OP::OUTPUT), acc_ref_gain_y->getInputPort<float>(Multiply::IP::INPUT_0));
        
        // connecting x control sys blocks
        outer_sys->connect(pos_h_demux->getOutputPort<float>(Demux3::OP::X), pid_x->getInputPort<float>(PID_Block::IP::ERROR));
        outer_sys->connect(vel_h_demux->getOutputPort<float>(Demux3::OP::X), pid_x->getInputPort<float>(PID_Block::IP::PV_DOT));
        outer_sys->connect(acc_h_demux->getOutputPort<float>(Demux3::OP::X), sum_acc_x->getInputPort<float>(Sum::OPERAND1));
        outer_sys->connect(pid_x->getOutputPort<float>(PID_Block::OP::COMMAND), sum_acc_x->getInputPort<float>(Sum::OPERAND2));
        outer_sys->connect(sum_acc_x->getOutputPort<float>(Sum::OP::OUTPUT), mrft_sw_x->getInputPort<float>(InvertedSwitch::IP::NC));
        outer_sys->connect(pid_x->getOutputPort<float>(PID_Block::OP::COMMAND), med_filt_x->getInputPort<float>(MedianFilter::IP::INPUT));
        outer_sys->connect(med_filt_x->getOutputPort<float>(MedianFilter::OP::OUTPUT), mrft_x->getInputPort<float>(MRFT_Block::IP::BIAS));
        outer_sys->connect(pos_h_demux->getOutputPort<float>(Demux3::OP::X), mrft_x->getInputPort<float>(MRFT_Block::IP::INPUT));
        outer_sys->connect(mrft_x->getOutputPort<float>(MRFT_Block::OP::COMMAND), mrft_sw_x->getInputPort<float>(InvertedSwitch::IP::NO));

        outer_sys->connect(med_filt_x->getOutputPort<float>(MedianFilter::OP::OUTPUT), hold_x_bias->getInputPort<float>(HoldVal::IP::INPUT));
        outer_sys->connect(hold_x_bias->getOutputPort<float>(HoldVal::OP::OUTPUT), sw_bias_x->getInputPort<float>(InvertedSwitch::IP::NO));
        outer_sys->connect(sw_bias_x->getOutputPort<float>(InvertedSwitch::OP::COM), sum_cmd_bias_x->getInputPort<float>(Sum::IP::OPERAND1));
        outer_sys->connect(mrft_sw_x->getOutputPort<float>(InvertedSwitch::OP::COM), sum_cmd_bias_x->getInputPort<float>(Sum::IP::OPERAND2));
        outer_sys->connect(sum_cmd_bias_x->getOutputPort<float>(Sum::OP::OUTPUT), mux_fh_des->getInputPort<float>(Mux3::IP::X));

        // connecting y control sys blocks
        outer_sys->connect(pos_h_demux->getOutputPort<float>(Demux3::OP::Y), pid_y->getInputPort<float>(PID_Block::IP::ERROR));
        outer_sys->connect(vel_h_demux->getOutputPort<float>(Demux3::OP::Y), pid_y->getInputPort<float>(PID_Block::IP::PV_DOT));
        outer_sys->connect(acc_h_demux->getOutputPort<float>(Demux3::OP::Y),  sum_acc_y->getInputPort<float>(Sum::OPERAND1));
        outer_sys->connect(pid_y->getOutputPort<float>(PID_Block::OP::COMMAND), sum_acc_y->getInputPort<float>(Sum::OPERAND2));
        outer_sys->connect(sum_acc_y->getOutputPort<float>(Sum::OP::OUTPUT), mrft_sw_y->getInputPort<float>(InvertedSwitch::IP::NC));
        outer_sys->connect(pid_y->getOutputPort<float>(PID_Block::OP::COMMAND), med_filt_y->getInputPort<float>(MedianFilter::IP::INPUT));
        outer_sys->connect(med_filt_y->getOutputPort<float>(MedianFilter::OP::OUTPUT), mrft_y->getInputPort<float>(MRFT_Block::IP::BIAS));
        outer_sys->connect(pos_h_demux->getOutputPort<float>(Demux3::OP::Y), mrft_y->getInputPort<float>(MRFT_Block::IP::INPUT));
        outer_sys->connect(mrft_y->getOutputPort<float>(MRFT_Block::OP::COMMAND), mrft_sw_y->getInputPort<float>(InvertedSwitch::IP::NO));

        outer_sys->connect(med_filt_y->getOutputPort<float>(MedianFilter::OP::OUTPUT), hold_y_bias->getInputPort<float>(HoldVal::IP::INPUT));
        outer_sys->connect(hold_y_bias->getOutputPort<float>(HoldVal::OP::OUTPUT), sw_bias_y->getInputPort<float>(InvertedSwitch::IP::NO));
        outer_sys->connect(sw_bias_y->getOutputPort<float>(InvertedSwitch::OP::COM), sum_cmd_bias_y->getInputPort<float>(Sum::IP::OPERAND1));
        outer_sys->connect(mrft_sw_y->getOutputPort<float>(InvertedSwitch::OP::COM), sum_cmd_bias_y->getInputPort<float>(Sum::IP::OPERAND2));
        outer_sys->connect(sum_cmd_bias_y->getOutputPort<float>(Sum::OP::OUTPUT), mux_fh_des->getInputPort<float>(Mux3::IP::Y));

        // connecting z control sys blocks
        outer_sys->connect(pos_h_demux->getOutputPort<float>(Demux3::OP::Z), pid_z->getInputPort<float>(PID_Block::IP::ERROR));
        outer_sys->connect(vel_h_demux->getOutputPort<float>(Demux3::OP::Z), pid_z->getInputPort<float>(PID_Block::IP::PV_DOT));
        outer_sys->connect(pid_z->getOutputPort<float>(PID_Block::OP::COMMAND), sum_bias_z->getInputPort<float>(Sum::IP::OPERAND1));
        outer_sys->connect(bias_z->getOutputPort<float>(Constant<float>::OP::OUTPUT), sum_bias_z->getInputPort<float>(Sum::IP::OPERAND2));
        outer_sys->connect(sum_bias_z->getOutputPort<float>(Sum::OP::OUTPUT), sat_z->getInputPort<float>(Saturation::IP::INPUT));
        outer_sys->connect(sat_z->getOutputPort<float>(Saturation::OP::OUTPUT), mrft_sw_z->getInputPort<float>(InvertedSwitch::IP::NC));
        outer_sys->connect(sat_z->getOutputPort<float>(Saturation::OP::OUTPUT), med_filt_z->getInputPort<float>(MedianFilter::IP::INPUT));
        outer_sys->connect(med_filt_z->getOutputPort<float>(MedianFilter::OP::OUTPUT), mrft_z->getInputPort<float>(MRFT_Block::IP::BIAS));
        outer_sys->connect(pos_h_demux->getOutputPort<float>(Demux3::OP::Z), mrft_z->getInputPort<float>(MRFT_Block::IP::INPUT));
        outer_sys->connect(mrft_z->getOutputPort<float>(MRFT_Block::OP::COMMAND), mrft_sw_z->getInputPort<float>(InvertedSwitch::IP::NO));
        outer_sys->connect(mrft_sw_z->getOutputPort<float>(InvertedSwitch::OP::COM), mux_fh_des->getInputPort<float>(Mux3::IP::Z));

        // connecting feedback linearization blocks
        outer_sys->connect(mux_fh_des->getOutputPort<Vector3D<float>>(Mux3::OP::OUTPUT), fh2fi->getInputPort<Vector3D<float>>(FromHorizon::IP::INP_VEC));
        outer_sys->connect(demux_ori->getOutputPort<float>(Demux3::OP::Z), fh2fi->getInputPort<float>(FromHorizon::IP::YAW));
        outer_sys->connect(fh2fi->getOutputPort<Vector3D<float>>(FromHorizon::OP::OUT_VEC), f2rot->getInputPort<Vector3D<float>>(FbLinearizer::Force2Rot::IP::FORCE_I_DES));
        outer_sys->createSub("/waypoint_reference/yaw", f2rot->getInputPort<float>(FbLinearizer::Force2Rot::IP::YAW_REF));
        outer_sys->connect(f2rot->getOutputPort<tf2::Matrix3x3>(FbLinearizer::Force2Rot::OP::ROT_DES), rot2eul->getInputPort<tf2::Matrix3x3>(Rot2Eul::IP::ROT_MAT));

        // connecting ref angles data preparation blocks
        outer_sys->connect(rot2eul->getOutputPort<Vector3D<float>>(Rot2Eul::OP::EUL_ANGLES), demux_eul_des->getInputPort<Vector3D<float>>(Demux3::IP::INPUT));
        outer_sys->connect(demux_eul_des->getOutputPort<float>(Demux3::OP::X), sat_x->getInputPort<float>(Saturation::IP::INPUT));
        outer_sys->connect(demux_eul_des->getOutputPort<float>(Demux3::OP::Y), sat_y->getInputPort<float>(Saturation::IP::INPUT));
        outer_sys->connect(sat_x->getOutputPort<float>(Saturation::OP::OUTPUT), mux_eul_des->getInputPort<float>(Mux3::IP::X));
        outer_sys->connect(sat_y->getOutputPort<float>(Saturation::OP::OUTPUT), mux_eul_des->getInputPort<float>(Mux3::IP::Y));
        outer_sys->connect(demux_eul_des->getOutputPort<float>(Demux3::OP::Z), mux_eul_des->getInputPort<float>(Mux3::IP::Z));

        ///////////////// configuring publishers //////////////////
        outer_sys->createPub<Vector3D<float>>(TYPE::Float3, "/pos_err_h", to_horizon_pos->getOutputPort<Vector3D<float>>(ToHorizon::OUT_VEC));
        outer_sys->createPub<Vector3D<float>>(TYPE::Float3, "/fh_des", mux_fh_des->getOutputPort<Vector3D<float>>(Mux3::OP::OUTPUT));
        outer_sys->createPub<Vector3D<float>>(TYPE::Float3, "/fi_des", fh2fi->getOutputPort<Vector3D<float>>(FromHorizon::OP::OUT_VEC));
        outer_sys->createPub<Vector3D<float>>(TYPE::Float3, "/rot_des", mux_eul_des->getOutputPort<Vector3D<float>>(Mux3::OP::OUTPUT));
        // outer_sys->createPub( TYPE::Float3, "/vel_h_x", diff_pos->getOutputPort<Vector3D<float>>(0));
        outer_sys->createPub( TYPE::Float3, "/vel_h_filt", pos_filt->getOutputPort<Vector3D<float>>(0));


        // configuring yaw provider for mission scenario
        auto mux_yaw = outer_sys->createBlock(BLOCK_ID::MUX3, "Mux_Yaw");
        outer_sys->connect( demux_ori->getOutputPort<float>(Demux3::OP::Z), mux_yaw->getInputPort<float>(Mux3::IP::X));
        outer_sys->createPub( TYPE::Float3, "/providers/yaw", mux_yaw->getOutputPort<Vector3D<float>>(Mux3::OP::OUTPUT));

        /////////////////// Setting External Triggers /////////////////

        //setting filter trigger
        auto enable_bwfilt_trig = outer_sys->createUpdateTrigger(UPDATE_MSG_TYPE::BOOL_MSG, "/enable_outer_filter");
        outer_sys->connectExternalTrigger(enable_bwfilt_trig, pos_filt);

        // setting pid controller triggers
        auto rest_all_trig = outer_sys->createResetTrigger("reset_outer_controller");
        outer_sys->connectExternalTrigger(rest_all_trig, pid_x);
        outer_sys->connectExternalTrigger(rest_all_trig, pid_y);
        outer_sys->connectExternalTrigger(rest_all_trig, pid_z);
        outer_sys->createUpdateTrigger(UPDATE_MSG_TYPE::BOOL_MSG, "/pid_z_trig", pid_z);
        auto update_pid_trig = outer_sys->createUpdateTrigger(UPDATE_MSG_TYPE::PID_UPDATE, "/update_controller/pid/outer");
        outer_sys->connectExternalTrigger(update_pid_trig, pid_x);
        outer_sys->connectExternalTrigger(update_pid_trig, pid_y);
        outer_sys->connectExternalTrigger(update_pid_trig, pid_z);

        // setting hold thrust val trigger
        // outer_sys->createUpdateTrigger(UPDATE_MSG_TYPE::BOOL_MSG, "/record_hover_thrust", hold_thrust_val);

        // MRFT X triggering configuration
        trig_srv_x = new ROSUnit_MRFTSwitchSrv(nh, "mrft_switch_x");
        auto pid_x_trig = trig_srv_x->getPIDTrig(); outer_sys->addExternalTrigger(pid_x_trig, "Pid_Trig_x");
        auto mrft_x_trig = trig_srv_x->getMRFTTrig(); outer_sys->addExternalTrigger(mrft_x_trig, "Mrft_Trig_x");
        auto mrft_sw_x_trig = trig_srv_x->registerSwitchTrig(); outer_sys->addExternalTrigger(mrft_sw_x_trig, "Mrft_Sw_Trig_x");
        outer_sys->connectExternalTrigger(pid_x_trig, pid_x);
        outer_sys->connectExternalTrigger(mrft_x_trig, mrft_x);
        outer_sys->connectExternalTrigger(mrft_sw_x_trig, mrft_sw_x);

        // MRFT Y triggering configuration
        trig_srv_y = new ROSUnit_MRFTSwitchSrv(nh, "mrft_switch_y");
        auto pid_y_trig = trig_srv_y->getPIDTrig(); outer_sys->addExternalTrigger(pid_y_trig, "Pid_Trig_y");
        auto mrft_y_trig = trig_srv_y->getMRFTTrig(); outer_sys->addExternalTrigger(mrft_y_trig, "Mrft_Trig_y");
        auto mrft_sw_y_trig = trig_srv_y->registerSwitchTrig(); outer_sys->addExternalTrigger(mrft_sw_y_trig, "Mrft_Sw_Trig_y");
        outer_sys->connectExternalTrigger(pid_y_trig, pid_y);
        outer_sys->connectExternalTrigger(mrft_y_trig, mrft_y);
        outer_sys->connectExternalTrigger(mrft_sw_y_trig, mrft_sw_y);

        // MRFT Z triggering configuration
        trig_srv_z = new ROSUnit_MRFTSwitchSrv(nh, "mrft_switch_z");
        auto pid_z_trig = trig_srv_z->getPIDTrig(); outer_sys->addExternalTrigger(pid_z_trig, "Pid_Trig_z");
        auto mrft_z_trig = trig_srv_z->getMRFTTrig(); outer_sys->addExternalTrigger(mrft_z_trig, "Mrft_Trig_z");
        auto mrft_sw_z_trig = trig_srv_z->registerSwitchTrig(); outer_sys->addExternalTrigger(mrft_sw_z_trig, "Mrft_Sw_Trig_z");
        outer_sys->connectExternalTrigger(pid_z_trig, pid_z);
        outer_sys->connectExternalTrigger(mrft_z_trig, mrft_z);
        outer_sys->connectExternalTrigger(mrft_sw_z_trig, mrft_sw_z);

        // setting mrft controllers triggers        
        auto mrft_update_trig = outer_sys->createUpdateTrigger(UPDATE_MSG_TYPE::MRFT_UPDATE, "/update_controller/mrft/outer") ;
        outer_sys->connectExternalTrigger(mrft_update_trig, mrft_x);
        outer_sys->connectExternalTrigger(mrft_update_trig, mrft_y);
        outer_sys->connectExternalTrigger(mrft_update_trig, mrft_z);

        // setting slam provider switches
        auto vs_x_sw_trig = outer_sys->createUpdateTrigger(UPDATE_MSG_TYPE::BOOL_MSG, "/vs_x_switch");
        outer_sys->connectExternalTrigger(vs_x_sw_trig, pos_x_sw);
        outer_sys->connectExternalTrigger(vs_x_sw_trig, vel_x_sw);

        // setting slam provider switches
        auto vs_z_sw_trig = outer_sys->createUpdateTrigger(UPDATE_MSG_TYPE::BOOL_MSG, "/vs_z_switch");
        outer_sys->connectExternalTrigger(vs_z_sw_trig, pos_z_sw);
        outer_sys->connectExternalTrigger(vs_z_sw_trig, vel_z_sw);


        // adding xy bias correction triggers
        auto bias_corr_trig = outer_sys->createUpdateTrigger(UPDATE_MSG_TYPE::BOOL_MSG, "/correct_biases");
        outer_sys->connectExternalTrigger(bias_corr_trig, hold_x_bias);
        outer_sys->connectExternalTrigger(bias_corr_trig, hold_y_bias);
        outer_sys->connectExternalTrigger(bias_corr_trig, sw_bias_x);
        outer_sys->connectExternalTrigger(bias_corr_trig, sw_bias_y);

        /////////////// initializing and starting the outer loop system  /////////////////
        outer_sys->start();

    }
    
} // namespace HEAR
