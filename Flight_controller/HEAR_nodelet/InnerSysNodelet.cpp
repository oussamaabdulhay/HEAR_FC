
#include "InnerSysNodelet.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(HEAR::InnerSysNodelet, nodelet::Nodelet)

namespace HEAR
{
    InnerSysNodelet::~InnerSysNodelet(){
        delete inner_sys;
    }
    void InnerSysNodelet::onInit(){
        ros::NodeHandle nh(getNodeHandle());
        ros::NodeHandle pnh(getPrivateNodeHandle());
        inner_sys = new RosSystem(nh, pnh, FREQUENCY, "InnerLoop");

        // creating Blocks
        auto filt_angle_rate = inner_sys->createBlock(BLOCK_ID::BW_FILT2, "Filt_angle_rate", TYPE::Float3);
        ((BWFilter2<Vector3D<float>>*)filt_angle_rate)->setCoeff(BWFilt2_coeff::coeff_N200C60);
        auto demux_angle_rate = inner_sys->createBlock(BLOCK_ID::DEMUX3, "Demux_AngleRate");
        auto demux_ori = inner_sys->createBlock(BLOCK_ID::DEMUX3, "Demux_Ori");
        auto mux_rpy = inner_sys->createBlock(BLOCK_ID::MUX3, "Mux_Ori");
        auto eul2Rb_des = new Eul2Rot(0);
        inner_sys->addBlock(eul2Rb_des, "Eul_to_Rb_des");
        auto eul2Rb = new Eul2Rot(0);
        inner_sys->addBlock(eul2Rb, "Eul_to_Rb");
        auto roterr2angerr = new FbLinearizer::RotDiff2Rod(0);
        inner_sys->addBlock(roterr2angerr, "RotErr_to_AngErr");
        auto demux_angerr = inner_sys->createBlock(BLOCK_ID::DEMUX3, "Demux_AngleErr");
        auto negate_roll_dot = inner_sys->createBlock(BLOCK_ID::GAIN, "Negate_Roll_Dot"); ((Gain*)negate_roll_dot)->setGain(-1.0);
        auto negate_pitch_dot = inner_sys->createBlock(BLOCK_ID::GAIN, "Negate_Pitch_Dot"); ((Gain*)negate_pitch_dot)->setGain(-1.0);
        auto pid_roll = inner_sys->createBlock(BLOCK_ID::PID, "Pid_Roll"); ((PID_Block*)pid_roll)->setPID_ID(PID_ID::PID_ROLL); 
        auto pid_pitch = inner_sys->createBlock(BLOCK_ID::PID, "Pid_Pitch"); ((PID_Block*)pid_pitch)->setPID_ID(PID_ID::PID_PITCH);
        auto pid_yaw = inner_sys->createBlock(BLOCK_ID::PID, "Pid_Yaw"); ((PID_Block*)pid_yaw)->setPID_ID(PID_ID::PID_YAW);
        auto pid_yaw_rt = inner_sys->createBlock(BLOCK_ID::PID, "Pid_Yaw_Rate"); ((PID_Block*)pid_yaw_rt)->setPID_ID(PID_ID::PID_YAW_RATE);
        auto sum_ref_yaw_rt = inner_sys->createBlock(BLOCK_ID::SUM, "Sum_Ref_Yaw_rt");
        auto mux_angle_u = inner_sys->createBlock(BLOCK_ID::MUX3, "Mux_Angle_u");
        auto sat_yaw = inner_sys->createBlock(BLOCK_ID::SATURATION, "Sat_yaw"); ((Saturation*)sat_yaw)->setClipValue(YAW_SAT_VALUE);
        auto demux_yaw = inner_sys->createBlock(BLOCK_ID::DEMUX3, "Demux_Yaw");

        //////// creating MRFT specific blocks ////////////
        auto mrft_roll = inner_sys->createBlock(BLOCK_ID::MRFT, "Mrft_roll"); ((MRFT_Block*)mrft_roll)->setMRFT_ID(MRFT_ID::MRFT_ROLL);
        auto mrft_pitch = inner_sys->createBlock(BLOCK_ID::MRFT, "Mrft_pitch"); ((MRFT_Block*)mrft_pitch)->setMRFT_ID(MRFT_ID::MRFT_PITCH);
        auto mrft_sw_roll = inner_sys->createBlock(BLOCK_ID::INVERTED_SWITCH, "Mrft_Sw_roll");
        auto mrft_sw_pitch = inner_sys->createBlock(BLOCK_ID::INVERTED_SWITCH, "Mrft_Sw_pitch");
        auto med_filt_roll = inner_sys->createBlock(BLOCK_ID::MEDIAN_FILTER, "Med_Filt_roll"); ((MedianFilter*)med_filt_roll)->setWinSize(50);
        auto med_filt_pitch = inner_sys->createBlock(BLOCK_ID::MEDIAN_FILTER, "Med_Filt_pitch"); ((MedianFilter*)med_filt_pitch)->setWinSize(50);
        // auto ref_sw_roll = inner_sys->createBlock(BLOCK_ID::INVERTED_SWITCH, "Ref_Sw_roll");
        // auto ref_sw_pitch = inner_sys->createBlock(BLOCK_ID::INVERTED_SWITCH, "Ref_Sw_pitch");
        auto hold_ref_roll = inner_sys->createBlock(BLOCK_ID::HOLDVAL, "Hold_Ref_roll");
        auto hold_ref_pitch = inner_sys->createBlock(BLOCK_ID::HOLDVAL, "Hold_Ref_pitch");

        auto demux_rot_des = inner_sys->createBlock(BLOCK_ID::DEMUX3, "Demux_Rot_Des");
        auto mux_angle_ref = inner_sys->createBlock(BLOCK_ID::MUX3, "Mux_Angle_Ref");

        //connecting blocks
        providers = new ROSUnit_PoseProvider (nh);
        auto ori_port = inner_sys->createExternalInputPort<Vector3D<float>>("Ori_port");
        auto angle_rate_port = inner_sys->createExternalInputPort<Vector3D<float>>("Angle_rt_port");
        inner_sys->connectExternalInput(ori_port, providers->registerImuOri("/filter/quaternion"));
        inner_sys->connectExternalInput(ori_port, demux_ori->getInputPort<Vector3D<float>>(Demux3::IP::INPUT));
        inner_sys->connectExternalInput(angle_rate_port, providers->registerImuAngularRate("/imu/angular_velocity"));
        inner_sys->connectExternalInput(angle_rate_port, filt_angle_rate->getInputPort<Vector3D<float>>(0));
        inner_sys->connect(filt_angle_rate->getOutputPort<Vector3D<float>>(0), demux_angle_rate->getInputPort<Vector3D<float>>(Demux3::IP::INPUT));

        inner_sys->createSub( TYPE::Float3, "/rot_des", demux_rot_des->getInputPort<Vector3D<float>>(Demux3::IP::INPUT));
        inner_sys->connect(demux_rot_des->getOutputPort<float>(Demux3::OP::X), hold_ref_roll->getInputPort<float>(HoldVal::IP::INPUT));
        inner_sys->connect(demux_rot_des->getOutputPort<float>(Demux3::OP::Y), hold_ref_pitch->getInputPort<float>(HoldVal::IP::INPUT));
        // inner_sys->connect(demux_ori->getOutputPort<float>(Demux3::OP::X), );
        // inner_sys->connect(demux_ori->getOutputPort<float>(Demux3::OP::Y), );
        // inner_sys->connect(, ref_sw_roll->getInputPort<float>(InvertedSwitch::IP::NO));
        // inner_sys->connect(, ref_sw_pitch->getInputPort<float>(InvertedSwitch::IP::NO));

        inner_sys->connect(hold_ref_roll->getOutputPort<float>(HoldVal::OP::OUTPUT), mux_angle_ref->getInputPort<float>(Mux3::IP::X));
        inner_sys->connect(hold_ref_pitch->getOutputPort<float>(HoldVal::OP::OUTPUT), mux_angle_ref->getInputPort<float>(Mux3::IP::Y));
        inner_sys->connect(demux_rot_des->getOutputPort<float>(Demux3::OP::Z), mux_angle_ref->getInputPort<float>(Mux3::IP::Z));
        inner_sys->connect(mux_angle_ref->getOutputPort<Vector3D<float>>(Mux3::OP::OUTPUT), eul2Rb_des->getInputPort<Vector3D<float>>(Eul2Rot::IP::EUL_ANGLES));
        inner_sys->createSub( TYPE::Float3, "/providers/yaw", demux_yaw->getInputPort<Vector3D<float>>(Demux3::IP::INPUT));
        inner_sys->connect(demux_yaw->getOutputPort<float>(Demux3::OP::X), mux_rpy->getInputPort<float>(Mux3::IP::Z));
        inner_sys->connect(demux_ori->getOutputPort<float>(Demux3::OP::X), mux_rpy->getInputPort<float>(Mux3::IP::X));
        inner_sys->connect(demux_ori->getOutputPort<float>(Demux3::OP::Y), mux_rpy->getInputPort<float>(Mux3::IP::Y));
        inner_sys->connect(mux_rpy->getOutputPort<Vector3D<float>>(Mux3::OP::OUTPUT), eul2Rb->getInputPort<Vector3D<float>>(Eul2Rot::IP::EUL_ANGLES));

        inner_sys->createSub( TYPE::Float3,"/fi_des", roterr2angerr->getInputPort<Vector3D<float>>(FbLinearizer::RotDiff2Rod::IP::F_IDES));
        inner_sys->connect(eul2Rb_des->getOutputPort<tf2::Matrix3x3>(Eul2Rot::OP::ROT_MAT), roterr2angerr->getInputPort<tf2::Matrix3x3>(FbLinearizer::RotDiff2Rod::IP::R_BDES_I));
        inner_sys->connect(eul2Rb->getOutputPort<tf2::Matrix3x3>(Eul2Rot::OP::ROT_MAT), roterr2angerr->getInputPort<tf2::Matrix3x3>(FbLinearizer::RotDiff2Rod::IP::R_I_B));
        inner_sys->connect(roterr2angerr->getOutputPort<Vector3D<float>>(FbLinearizer::RotDiff2Rod::OP::ROD_ANGLES), demux_angerr->getInputPort<Vector3D<float>>(Demux3::IP::INPUT));

        inner_sys->connect(demux_angerr->getOutputPort<float>(Demux3::OP::X), pid_roll->getInputPort<float>(PID_Block::IP::ERROR));
        inner_sys->connect(demux_angle_rate->getOutputPort<float>(Demux3::OP::X), negate_roll_dot->getInputPort<float>(Gain::IP::INPUT));
        inner_sys->connect(negate_roll_dot->getOutputPort<float>(Gain::OP::OUTPUT), pid_roll->getInputPort<float>(PID_Block::IP::PV_DOT));
        inner_sys->connect(pid_roll->getOutputPort<float>(PID_Block::OP::COMMAND), med_filt_roll->getInputPort<float>(MedianFilter::IP::INPUT));
        inner_sys->connect(med_filt_roll->getOutputPort<float>(MedianFilter::OP::OUTPUT), mrft_roll->getInputPort<float>(MRFT_Block::IP::BIAS));                
        inner_sys->connect(demux_angerr->getOutputPort<float>(Demux3::OP::X), mrft_roll->getInputPort<float>(MRFT_Block::IP::INPUT));
        inner_sys->connect(demux_angerr->getOutputPort<float>(Demux3::OP::Y), pid_pitch->getInputPort<float>(PID_Block::IP::ERROR));
        inner_sys->connect(demux_angle_rate->getOutputPort<float>(Demux3::OP::Y), negate_pitch_dot->getInputPort<float>(Gain::IP::INPUT));
        inner_sys->connect(negate_pitch_dot->getOutputPort<float>(Gain::OP::OUTPUT), pid_pitch->getInputPort<float>(PID_Block::IP::PV_DOT));
        inner_sys->connect(pid_pitch->getOutputPort<float>(PID_Block::OP::COMMAND), med_filt_pitch->getInputPort<float>(MedianFilter::IP::INPUT));
        inner_sys->connect(med_filt_pitch->getOutputPort<float>(MedianFilter::OP::OUTPUT), mrft_pitch->getInputPort<float>(MRFT_Block::IP::BIAS));                
        inner_sys->connect(demux_angerr->getOutputPort<float>(Demux3::OP::Y), mrft_pitch->getInputPort<float>(MRFT_Block::IP::INPUT));

        inner_sys->connect(demux_angerr->getOutputPort<float>(Demux3::OP::Z), pid_yaw->getInputPort<float>(PID_Block::IP::ERROR));

        inner_sys->connect(pid_yaw->getOutputPort<float>(PID_Block::OP::COMMAND), sat_yaw->getInputPort<float>(Saturation::IP::INPUT));
        inner_sys->connect(sat_yaw->getOutputPort<float>(Saturation::OP::OUTPUT), sum_ref_yaw_rt->getInputPort<float>(Sum::IP::OPERAND1));
        inner_sys->connect(demux_angle_rate->getOutputPort<float>(Demux3::OP::Z), sum_ref_yaw_rt->getInputPort<float>(Sum::IP::OPERAND2));
        inner_sys->connect(sum_ref_yaw_rt->getOutputPort<float>(Sum::OP::OUTPUT), pid_yaw_rt->getInputPort<float>(PID_Block::IP::ERROR));

        inner_sys->connect(pid_roll->getOutputPort<float>(PID_Block::OP::COMMAND), mrft_sw_roll->getInputPort<float>(InvertedSwitch::IP::NC));
        inner_sys->connect(mrft_roll->getOutputPort<float>(MRFT_Block::OP::COMMAND), mrft_sw_roll->getInputPort<float>(InvertedSwitch::IP::NO));
        inner_sys->connect(mrft_sw_roll->getOutputPort<float>(InvertedSwitch::OP::COM), mux_angle_u->getInputPort<float>(Mux3::IP::X));
        inner_sys->connect(pid_pitch->getOutputPort<float>(PID_Block::OP::COMMAND), mrft_sw_pitch->getInputPort<float>(InvertedSwitch::IP::NC));
        inner_sys->connect(mrft_pitch->getOutputPort<float>(MRFT_Block::OP::COMMAND), mrft_sw_pitch->getInputPort<float>(InvertedSwitch::IP::NO));
        inner_sys->connect(mrft_sw_pitch->getOutputPort<float>(InvertedSwitch::OP::COM), mux_angle_u->getInputPort<float>(Mux3::IP::Y));
        inner_sys->connect(pid_yaw_rt->getOutputPort<float>(PID_Block::OP::COMMAND), mux_angle_u->getInputPort<float>(Mux3::IP::Z));

        inner_sys->createPub(TYPE::Float3, "/angle_u", mux_angle_u->getOutputPort<Vector3D<float>>(Mux3::OP::OUTPUT));
        inner_sys->createPub(TYPE::Float3, "/angle_ref", mux_angle_ref->getOutputPort<Vector3D<float>>(Mux3::OP::OUTPUT));
        inner_sys->createPub(TYPE::Float3, "/angle_err", roterr2angerr->getOutputPort<Vector3D<float>>(FbLinearizer::RotDiff2Rod::OP::ROD_ANGLES));
        inner_sys->createPub( "/thrust_cmd", roterr2angerr->getOutputPort<float>(FbLinearizer::RotDiff2Rod::OP::THRUST));
        inner_sys->createPub(TYPE::Float3, "body_ori", mux_rpy->getOutputPort<Vector3D<float>>(Mux3::OP::OUTPUT));
        inner_sys->createPub(TYPE::Float3, "filtered_angle_rt", filt_angle_rate->getOutputPort<Vector3D<float>>(0));

        auto update_pid_trig = inner_sys->createUpdateTrigger(UPDATE_MSG_TYPE::PID_UPDATE, "/update_controller/pid/inner");
        inner_sys->connectExternalTrigger(update_pid_trig, pid_roll);
        inner_sys->connectExternalTrigger(update_pid_trig, pid_pitch);
        inner_sys->connectExternalTrigger(update_pid_trig, pid_yaw);
        inner_sys->connectExternalTrigger(update_pid_trig, pid_yaw_rt);

        auto enable_bwfilt_trig = inner_sys->createUpdateTrigger(UPDATE_MSG_TYPE::BOOL_MSG, "/enable_inner_filter");
        inner_sys->connectExternalTrigger(enable_bwfilt_trig, filt_angle_rate);
        auto mrft_update_trig = inner_sys->createUpdateTrigger(UPDATE_MSG_TYPE::MRFT_UPDATE, "/update_controller/mrft/inner") ;
        inner_sys->connectExternalTrigger(mrft_update_trig, mrft_roll);
        inner_sys->connectExternalTrigger(mrft_update_trig, mrft_pitch);

        trig_srv_roll = new ROSUnit_MRFTSwitchSrv(nh, "mrft_switch_roll");
        auto pid_roll_trig = trig_srv_roll->getPIDTrig(); inner_sys->addExternalTrigger(pid_roll_trig, "Pid_Trig_roll");
        auto mrft_roll_trig = trig_srv_roll->getMRFTTrig(); inner_sys->addExternalTrigger(mrft_roll_trig, "Mrft_Trig_roll");
        auto mrft_sw_roll_trig = trig_srv_roll->registerSwitchTrig(); inner_sys->addExternalTrigger(mrft_sw_roll_trig, "Mrft_Sw_Trig_roll");
        inner_sys->connectExternalTrigger(pid_roll_trig, pid_roll);
        inner_sys->connectExternalTrigger(mrft_roll_trig, mrft_roll);
        inner_sys->connectExternalTrigger(mrft_roll_trig, hold_ref_roll);
        inner_sys->connectExternalTrigger(mrft_sw_roll_trig, mrft_sw_roll);
        // inner_sys->connectExternalTrigger(mrft_sw_roll_trig, ref_sw_roll);

        trig_srv_pitch = new ROSUnit_MRFTSwitchSrv(nh, "mrft_switch_pitch");
        auto pid_pitch_trig = trig_srv_pitch->getPIDTrig(); inner_sys->addExternalTrigger(pid_pitch_trig, "Pid_Trig_pitch");
        auto mrft_pitch_trig = trig_srv_pitch->getMRFTTrig(); inner_sys->addExternalTrigger(mrft_pitch_trig, "Mrft_Trig_pitch");
        auto mrft_sw_pitch_trig = trig_srv_pitch->registerSwitchTrig(); inner_sys->addExternalTrigger(mrft_sw_pitch_trig, "Mrft_Sw_Trig_pitch");
        inner_sys->connectExternalTrigger(pid_pitch_trig, pid_pitch);
        inner_sys->connectExternalTrigger(mrft_pitch_trig, mrft_pitch);
        inner_sys->connectExternalTrigger(mrft_pitch_trig, hold_ref_pitch);
        inner_sys->connectExternalTrigger(mrft_sw_pitch_trig, mrft_sw_pitch);
        // inner_sys->connectExternalTrigger(mrft_sw_pitch_trig, ref_sw_pitch);

        inner_sys->start();

    }
    
} // namespace HEAR
