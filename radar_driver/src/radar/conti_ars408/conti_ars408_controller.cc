#include "conti_ars408_controller.h"
#include <pb_msgs/utils.h>
#ifndef NVIDIA_DRIVE
    #include "can_interface/kvaser_interface.h"
#else
    #include "can_interface/mttcan_interface.h"
#endif

namespace radar_driver {
ContiController::ContiController(const CANCardParameter &param) { param_ = param; }

void ContiController::init(ros::NodeHandle &nh) {
    is_running_ = true;
    #ifndef NVIDIA_DRIVE
        can_reader_.reset(new radar_driver::KvaserInterface());
        can_writer_.reset(new radar_driver::KvaserInterface());
    #else
        can_reader_.reset(new radar_driver::MttcanInterface());
        can_writer_.reset(new radar_driver::MttcanInterface());
    #endif
    can_reader_->Init(param_);
    can_reader_->Start();
    can_writer_->Init(param_);
    can_writer_->Start();
    fbk_frame32_updated_ = false;
    fbk_frame35_updated_ = false;
    /*
    can_cmd_ = nh.subscribe("/saicic/motion_planners/control_command", 100,
                            &GemController::EncodeMsgCallback, this);
    can_cmd_aux_ = nh.subscribe("/saicic/motion_planners/control_BCM", 100,
                                &GemController::EncodeMsgAuxCallback, this);
    can_fbk_ = nh.advertise<candata_msgs_pb::CANData>("/vehicle/canFeedback", 1);
    */
    can_recv_thread_.reset(new std::thread([this] { RecvThreadFunc(); }));
}

void ContiController::RecvThreadFunc() {
    const std::chrono::duration<double, std::micro> loop_pause{10};
    return_statuses ret;
    while (IsRunning()) {
        CanFrame frame;
        if ((ret = can_reader_->read(&frame)) == OK) {
            DecodeMsgPublish(frame);
        }
        if (ret != NO_MESSAGES_RECEIVED)
            continue;
        std::this_thread::sleep_for(loop_pause);
    }
}

void ContiController::DecodeMsgPublish(const CanFrame &frame) {
    switch (frame.id) {
        // Decode object information 
        // CAN frame id 32
        case 1546:
            conti_ars408_obj_0_status_unpack(&obj_status_, frame.data,sizeof(frame.data));    
            if (conti_ars408_obj_0_status_obj_nof_objects_is_in_range(obj_status_.obj_nof_objects))
            {
                
                //can_msg_.set
            }


        // CAN frame id 32: LLC_MotionFeedback1
        case 32:
            motivo_rexus_llc_motion_feedback1_unpack(&motion_fbk1_, frame.data,
                                                    sizeof(frame.data));
            if (motivo_rexus_llc_motion_feedback1_llc_fbk_state_is_in_range(
                motion_fbk1_.llc_fbk_state)) {
                int vcu_state_fbk =
                    int(motivo_rexus_llc_motion_feedback1_llc_fbk_state_decode(
                    motion_fbk1_.llc_fbk_state));
                switch (vcu_state_fbk) {
                    case 1:
                        can_msg_.set_vcu_state(candata_msgs_pb::CANData::AUTONOMY_NOT_ALLOWED);
                        break;
                    case 2:
                        can_msg_.set_vcu_state(candata_msgs_pb::CANData::AUTONOMY_ALLOWED);
                        break;
                    case 3:
                        can_msg_.set_vcu_state(candata_msgs_pb::CANData::AUTONOMY_REQUESTED);
                        break;
                    case 4:
                        can_msg_.set_vcu_state(candata_msgs_pb::CANData::AD_ENGAGED);
                        break;
                    case 13:
                        can_msg_.set_vcu_state(candata_msgs_pb::CANData::DISENGAGE_REQUESTED);
                        break;
                    case 14:
                        can_msg_.set_vcu_state(candata_msgs_pb::CANData::DISENGAGED);
                        break;
                    case 15:
                        can_msg_.set_vcu_state(candata_msgs_pb::CANData::VCU_FAULT);
                        break;
                    default:
                        can_msg_.set_vcu_state(candata_msgs_pb::CANData::VCU_OFF);
                        break;
                }
            }
            if (motivo_rexus_llc_motion_feedback1_llc_fbk_vehicle_velocity_is_in_range(
                motion_fbk1_.llc_fbk_vehicle_velocity)) {
                can_msg_.set_velocity(
                    motivo_rexus_llc_motion_feedback1_llc_fbk_vehicle_velocity_decode(
                        motion_fbk1_.llc_fbk_vehicle_velocity));
            }
            if (motivo_rexus_llc_motion_feedback1_llc_fbk_vehicle_acceleration_is_in_range(
                motion_fbk1_.llc_fbk_vehicle_acceleration)) {
                can_msg_.set_acc(
                    motivo_rexus_llc_motion_feedback1_llc_fbk_vehicle_acceleration_decode(
                        motion_fbk1_.llc_fbk_vehicle_acceleration));
            }
            if (motivo_rexus_llc_motion_feedback1_llc_fbk_steering_angle_is_in_range(
                motion_fbk1_.llc_fbk_steering_angle)) {
                can_msg_.set_steering_wheel_angle_deg(
                    -motivo_rexus_llc_motion_feedback1_llc_fbk_steering_angle_decode(
                        motion_fbk1_.llc_fbk_steering_angle));
            }
            can_msg_.set_throttle_pedal_pcnt(
                motivo_rexus_llc_motion_feedback1_llc_fbk_throttle_position_decode(
                    motion_fbk1_.llc_fbk_throttle_position));
            can_msg_.set_brake_pedal_psi(
                motivo_rexus_llc_motion_feedback1_llc_fbk_brake_pressure_fb_decode(
                    motion_fbk1_.llc_fbk_brake_pressure_fb));
            fbk_frame32_updated_ = true;
            break;
        // CAN frame id 35: LLC_MotionFeedback2
        case 35:
            motivo_rexus_llc_motion_feedback2_unpack(&motion_fbk2_, frame.data,
                                                        sizeof(frame.data));
            if (motivo_rexus_llc_motion_feedback2_llc_fbk_steering_curvature_is_in_range(
                motion_fbk2_.llc_fbk_steering_curvature)) {
                can_msg_.set_curvature(
                    -motivo_rexus_llc_motion_feedback2_llc_fbk_steering_curvature_decode(
                        motion_fbk2_.llc_fbk_steering_curvature));
            }
            fbk_frame35_updated_ = true;
            break;
    }
    // publish when frame 32 and 35 both update at least once
    if (fbk_frame32_updated_ && fbk_frame35_updated_) {
        fbk_frame32_updated_ = false;
        fbk_frame35_updated_ = false;
        *(can_msg_.mutable_header()->mutable_stamp()) =
            pb_msgs::utils::convert(ros::Time::now());
        can_msg_.mutable_header()->set_frame_id(param_.vehicle_licence_plate);
        can_fbk_.publish(can_msg_);
        can_msg_.Clear();
    }
}



/*

void ContiController::EncodeMsgCallback(const candata_msgs_pb::CANData &msg) {
    CanFrame frame;
    frame.dlc = 8;

    int longitudinal_control_mode = 0;
    int steering_control_mode = 0;
    if (msg.has_control_mode()) {
        switch (msg.control_mode()) {
            case candata_msgs_pb::CANData::SPEED_CURVATURE:
                longitudinal_control_mode = 1;
                steering_control_mode = 2;
                break;
            case candata_msgs_pb::CANData::SPEED_SLOPE_CURVATURE:
                longitudinal_control_mode = 1;
                steering_control_mode = 2;
                break;
            case candata_msgs_pb::CANData::STEER_ACC_DEC:
                longitudinal_control_mode = 2;
                steering_control_mode = 1;
                break;
            case candata_msgs_pb::CANData::STEER_ACC_DEC_TORQUE:
                longitudinal_control_mode = 2;
                steering_control_mode = 1;
                break;
            case candata_msgs_pb::CANData::STEER_THROTTLE_BRAKE_PEDAL:
                longitudinal_control_mode = 3;
                steering_control_mode = 1;
                break;
            case candata_msgs_pb::CANData::TORQUE_DEC_CURVATURE:
                longitudinal_control_mode = 2;
                steering_control_mode = 2;
                break;
            case candata_msgs_pb::CANData::THROTTLE_BRAKE_CURVATURE:
                longitudinal_control_mode = 3;
                steering_control_mode = 2;
                break;
            case candata_msgs_pb::CANData::ACC_DEC_CURVATURE:
                longitudinal_control_mode = 2;
                steering_control_mode = 2;
                break;
            case candata_msgs_pb::CANData::SPEED_STEER:
                longitudinal_control_mode = 1;
                steering_control_mode = 1;
                break;
            default:
                longitudinal_control_mode = 0;
                steering_control_mode = 0;
                break;
        }
    }

    // message id 16, ADC_MotionControl1
    if (msg.has_control_mode()) {
        struct motivo_rexus_adc_motion_control1_t cmd16;
        cmd16.adc_cmd_autonomy_state =
            motivo_rexus_adc_motion_control1_adc_cmd_autonomy_state_encode(1.0);
        cmd16.adc_cmd_steering_control_mode =
            motivo_rexus_adc_motion_control1_adc_cmd_steering_control_mode_encode(
                steering_control_mode);
        cmd16.adc_cmd_longitudinal_control_mode =
            motivo_rexus_adc_motion_control1_adc_cmd_longitudinal_control_mode_encode(
                longitudinal_control_mode);
        cmd16.adc_motion_control_counter =
            motivo_rexus_adc_motion_control1_adc_motion_control_counter_encode(
                (motion_control_counter_++) % 4);
        cmd16.adc_cmd_steer_wheel_angle =
            -motivo_rexus_adc_motion_control1_adc_cmd_steer_wheel_angle_encode(
                msg.steering_wheel_angle_deg());
        cmd16.adc_cmd_vehicle_velocity =
            motivo_rexus_adc_motion_control1_adc_cmd_vehicle_velocity_encode(
                msg.velocity());
        cmd16.adc_cmd_vehicle_acceleration =
            motivo_rexus_adc_motion_control1_adc_cmd_vehicle_acceleration_encode(
                msg.acc());
        cmd16.adc_cmd_throttle_position =
            motivo_rexus_adc_motion_control1_adc_cmd_throttle_position_encode(
                msg.throttle_pedal_pcnt());
        cmd16.adc_cmd_brake_pressure =
            motivo_rexus_adc_motion_control1_adc_cmd_brake_pressure_encode(
                msg.brake_pedal_psi());
        cmd16.adc_motion_control_security =
            motivo_rexus_adc_motion_control1_adc_motion_control_security_encode(
                0.0);
        motivo_rexus_adc_motion_control1_pack(frame.data, &cmd16,
                                                sizeof(frame.data));
        frame.id = 16;
        SendFunc(&frame);
    }

    // message id 17, ADC_MotionCtrlLim
    // send message 17 regardless of incoming control rihgt now
    struct motivo_rexus_adc_motion_ctrl_lim_t cmd17;
    cmd17.adc_cmd_steer_wheel_angle_limit =
        motivo_rexus_adc_motion_ctrl_lim_adc_cmd_steer_wheel_angle_limit_encode(
            600.0);
    cmd17.adc_cmd_steering_rate =
        motivo_rexus_adc_motion_ctrl_lim_adc_cmd_steering_rate_encode(400.0);
    cmd17.adc_cmd_throttle_command_limit =
        motivo_rexus_adc_motion_ctrl_lim_adc_cmd_throttle_command_limit_encode(
            100.0);
    cmd17.adc_cmd_vehicle_lateral_acc_limit =
        motivo_rexus_adc_motion_ctrl_lim_adc_cmd_vehicle_lateral_acc_limit_encode(
            10.0);
    cmd17.adc_cmd_vehicle_velocity_limit =
        motivo_rexus_adc_motion_ctrl_lim_adc_cmd_vehicle_velocity_limit_encode(
            25.0);
    cmd17.adc_cmd_vehicle_acceleration_limit =
        motivo_rexus_adc_motion_ctrl_lim_adc_cmd_vehicle_acceleration_limit_encode(
            10.0);
    cmd17.adc_cmd_vehicle_deceleration_limit =
        motivo_rexus_adc_motion_ctrl_lim_adc_cmd_vehicle_deceleration_limit_encode(
            10.0);
    motivo_rexus_adc_motion_ctrl_lim_pack(frame.data, &cmd17, sizeof(frame.data));
    frame.id = 17;
    SendFunc(&frame);

    // message id 18, ADC_MotionControl2
    if (msg.has_control_mode()) {
        struct motivo_rexus_adc_motion_control2_t cmd18;
        cmd18.adc_cmd_steering_curvature =
            motivo_rexus_adc_motion_control2_adc_cmd_steering_curvature_encode(
                -msg.curvature());
        motivo_rexus_adc_motion_control2_pack(frame.data, &cmd18,
                                            sizeof(frame.data));
        frame.id = 18;
        SendFunc(&frame);
    }
}


void ContiController::EncodeMsgAuxCallback(const candata_msgs_pb::Auxiliary &msg) {
    CanFrame frame;
    frame.dlc = 8;
    frame.id = 272;

    struct motivo_rexus_adc_auxiliary_control_t cmd272;
    if (msg.turn_signal() == candata_msgs_pb::Auxiliary::LEFT_TURN_SIGNAL) {
    cmd272.adc_cmd_turn_signal =
        motivo_rexus_adc_auxiliary_control_adc_cmd_turn_signal_encode(1.0);
    }
    else if (msg.turn_signal() == candata_msgs_pb::Auxiliary::RIGHT_TURN_SIGNAL) {
        cmd272.adc_cmd_turn_signal =
            motivo_rexus_adc_auxiliary_control_adc_cmd_turn_signal_encode(2.0);
    }
    else {
        cmd272.adc_cmd_turn_signal =
            motivo_rexus_adc_auxiliary_control_adc_cmd_turn_signal_encode(0.0);
    }
    cmd272.adc_cmd_horn =
        motivo_rexus_adc_auxiliary_control_adc_cmd_horn_encode(msg.horn());
    cmd272.adc_cmd_hazard_lights =
        motivo_rexus_adc_auxiliary_control_adc_cmd_hazard_lights_encode(msg.hazard_lights());
    if (msg.wiper() == candata_msgs_pb::Auxiliary::WIPER_OFF) {
        cmd272.adc_cmd_wiper =
            motivo_rexus_adc_auxiliary_control_adc_cmd_wiper_encode(false);
    }
    else {
        cmd272.adc_cmd_wiper =
            motivo_rexus_adc_auxiliary_control_adc_cmd_wiper_encode(true);
    }
    motivo_rexus_adc_auxiliary_control_pack(frame.data, &cmd272,
                                            sizeof(frame.data));
    SendFunc(&frame);
}
*/

}
 // namespace radar_driver
