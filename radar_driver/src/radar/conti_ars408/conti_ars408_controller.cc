#include "conti_ars408_controller.h"
//#include <pb_msgs/utils.h>
#include <radar_driver/ContiRaw.h>
#ifndef NVIDIA_DRIVE
    #include "can_interface/kvaser_interface.h"
#else
    #include "can_interface/mttcan_interface.h"
#endif

namespace radar_driver {
ContiController::ContiController(const CANCardParameter &param) { param_ = param; }

void ContiController::init(ros::NodeHandle &nh) {
    is_running_ = true;
    #ifndef NVIDIA_DRIVEcatkin_ws/src/msg/custom_msgs/CMakeLists.txt

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
    radar_60A_update_ = false;
    radar_60D_update_ = false;
    
    //fbk_frame32_updated_ = false;
    //fbk_frame35_updated_ = false;
    
    can_fbk_ = nh.advertise<radar_driver::ContiRaw>("radar_raw",1);
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
    }
}

void ContiController::DecodeMsgPublish(const CanFrame &frame) {
    switch (frame.id) {
        // Decode object information 
        // CAN frame id 60A (1546) : object list status 
        case 1546:
            conti_ars408_obj_0_status_unpack(&obj_0_status_, frame.data, sizeof(frame.data));    
            if (conti_ars408_obj_0_status_obj_nof_objects_is_in_range(obj_0_status_.obj_nof_objects)){
                can_msg_.objs_status.NofObjects = 
                conti_ars408_obj_0_status_obj_nof_objects_decode(obj_0_status_.obj_nof_objects);
            }
            if (conti_ars408_obj_0_status_obj_meas_counter_is_in_range(obj_0_status_.obj_meas_counter)){
                can_msg_.objs_status.MeasCounter = 
                conti_ars408_obj_0_status_obj_meas_counter_decode(obj_0_status_.obj_meas_counter);
            }
            if (conti_ars408_obj_0_status_obj_interface_version_is_in_range(obj_0_status_.obj_interface_version)){
                can_msg_.objs_status.InterfaceVersion = 
                conti_ars408_obj_0_status_obj_interface_version_decode(obj_0_status_.obj_interface_version);
            }
            can_msg_.objs_general.resize(0);
            can_msg_.objs_quality.resize(0);
            can_msg_.objs_extended.resize(0);
            radar_60A_update_ = true;
            break;

        // CAN frame id 60B (1547) : object general information 
        case 1547:
            {
                conti_ars408_obj_1_general_unpack(&obj_1_general_, frame.data, sizeof(frame.data));    
                radar_driver::Conti_obj_general obj_temp;
                if (conti_ars408_obj_1_general_obj_id_is_in_range(obj_1_general_.obj_id)){
                    obj_temp.ID = conti_ars408_obj_1_general_obj_id_decode(obj_1_general_.obj_id);
                } 
                if (conti_ars408_obj_1_general_obj_dist_long_is_in_range(obj_1_general_.obj_dist_long)){
                    obj_temp.DistLong = 
                    conti_ars408_obj_1_general_obj_dist_long_decode (obj_1_general_.obj_dist_long);
                }
                if (conti_ars408_obj_1_general_obj_dist_lat_is_in_range(obj_1_general_.obj_dist_lat)){
                    obj_temp.DistLat = 
                    conti_ars408_obj_1_general_obj_dist_lat_decode(obj_1_general_.obj_dist_lat);
                }
                if (conti_ars408_obj_1_general_obj_vrel_long_is_in_range(obj_1_general_.obj_vrel_long)){
                    obj_temp.VrelLong = 
                    conti_ars408_obj_1_general_obj_vrel_long_decode(obj_1_general_.obj_vrel_long);
                }
                if (conti_ars408_obj_1_general_obj_vrel_lat_is_in_range(obj_1_general_.obj_vrel_lat)){
                    obj_temp.VrelLat = 
                    conti_ars408_obj_1_general_obj_vrel_lat_decode(obj_1_general_.obj_vrel_lat);
                }
                if (conti_ars408_obj_1_general_obj_dyn_prop_is_in_range(obj_1_general_.obj_dyn_prop)){
                    obj_temp.DynProp = 
                    conti_ars408_obj_1_general_obj_dyn_prop_decode(obj_1_general_.obj_dyn_prop);
                }
                if (conti_ars408_obj_1_general_obj_rcs_is_in_range(obj_1_general_.obj_rcs)){
                    obj_temp.RCS = 
                    conti_ars408_obj_1_general_obj_rcs_decode(obj_1_general_.obj_rcs);
                }
                can_msg_.objs_general.push_back(obj_temp);
                std::cout << "Object general information: " << std::to_string(can_msg_.objs_general.back().DistLong) << std::endl;
            }
            break;

        // CAN frame id 60C (1548) : object quality information 
        case 1548:
            {
                conti_ars408_obj_2_quality_unpack(&obj_2_quality_, frame.data, sizeof(frame.data));
                radar_driver::Conti_obj_quality quality_temp;
                if (conti_ars408_obj_2_quality_obj_id_is_in_range(obj_2_quality_.obj_id)){
                    quality_temp.ID = 
                    conti_ars408_obj_2_quality_obj_id_decode(obj_2_quality_.obj_id);
                }
                if (conti_ars408_obj_2_quality_obj_dist_long_rms_is_in_range(obj_2_quality_.obj_dist_long_rms)){
                    quality_temp.DistLong_rms = 
                    conti_ars408_obj_2_quality_obj_dist_long_rms_decode(obj_2_quality_.obj_dist_long_rms);
                }
                if (conti_ars408_obj_2_quality_obj_vrel_long_rms_is_in_range(obj_2_quality_.obj_vrel_long_rms)){
                    quality_temp.VrelLong_rms = 
                    conti_ars408_obj_2_quality_obj_vrel_long_rms_decode(obj_2_quality_.obj_vrel_long_rms);
                }
                if (conti_ars408_obj_2_quality_obj_dist_lat_rms_is_in_range(obj_2_quality_.obj_dist_lat_rms)){
                    quality_temp.DisLat_rms = 
                    conti_ars408_obj_2_quality_obj_dist_lat_rms_decode(obj_2_quality_.obj_dist_lat_rms);
                }
                if (conti_ars408_obj_2_quality_obj_vrel_lat_rms_is_in_range(obj_2_quality_.obj_vrel_lat_rms)){
                    quality_temp.VrelLat_rms = 
                    conti_ars408_obj_2_quality_obj_vrel_lat_rms_decode(obj_2_quality_.obj_dist_lat_rms);
                }
                if (conti_ars408_obj_2_quality_obj_arel_lat_rms_is_in_range(obj_2_quality_.obj_arel_lat_rms)){
                    quality_temp.ArelLat_rms = 
                    conti_ars408_obj_2_quality_obj_arel_lat_rms_decode(obj_2_quality_.obj_arel_lat_rms);
                }
                if (conti_ars408_obj_2_quality_obj_arel_long_rms_is_in_range(obj_2_quality_.obj_arel_long_rms)){
                    quality_temp.ArelLong_rms = 
                    conti_ars408_obj_2_quality_obj_arel_long_rms_decode(obj_2_quality_.obj_arel_long_rms);
                }
                if (conti_ars408_obj_2_quality_obj_orientation_rms_is_in_range(obj_2_quality_.obj_orientation_rms)){
                    quality_temp.Orientation_rms = 
                    conti_ars408_obj_2_quality_obj_orientation_rms_decode(obj_2_quality_.obj_orientation_rms);
                }
                if (conti_ars408_obj_2_quality_obj_meas_state_is_in_range(obj_2_quality_.obj_meas_state)){
                    quality_temp.MeasState = 
                    conti_ars408_obj_2_quality_obj_meas_state_decode(obj_2_quality_.obj_meas_state);
                }
                if (conti_ars408_obj_2_quality_obj_prob_of_exist_is_in_range(obj_2_quality_.obj_prob_of_exist)){
                    quality_temp.ProbOfExist = 
                    conti_ars408_obj_2_quality_obj_prob_of_exist_decode(obj_2_quality_.obj_prob_of_exist);
                }
                can_msg_.objs_quality.push_back(quality_temp);
            }
            break;
        
        // CAN frame id 60D (1549) : object extended information
        case 1549:
            {
                conti_ars408_obj_3_extended_unpack(&obj_3_extended_, frame.data, sizeof(frame.data));
                radar_driver::Conti_obj_extended extended_temp;
                if (conti_ars408_obj_3_extended_obj_id_is_in_range(obj_3_extended_.obj_id)){
                    extended_temp.ID = 
                    conti_ars408_obj_3_extended_obj_id_decode(obj_3_extended_.obj_id);
                }
                if (conti_ars408_obj_3_extended_obj_arel_long_is_in_range(obj_3_extended_.obj_arel_long)){
                    extended_temp.ArelLong = 
                    conti_ars408_obj_3_extended_obj_arel_long_decode(obj_3_extended_.obj_arel_long);
                }
                if (conti_ars408_obj_3_extended_obj_class_is_in_range(obj_3_extended_.obj_class)){
                    extended_temp.Class = 
                    conti_ars408_obj_3_extended_obj_class_decode(obj_3_extended_.obj_class);
                }
                if (conti_ars408_obj_3_extended_obj_arel_lat_is_in_range(obj_3_extended_.obj_arel_lat)){
                    extended_temp.ArelLat = 
                    conti_ars408_obj_3_extended_obj_arel_lat_decode(obj_3_extended_.obj_arel_lat);
                }
                if (conti_ars408_obj_3_extended_obj_orientation_angle_is_in_range(obj_3_extended_.obj_orientation_angle)){
                    extended_temp.OrientationAngle = 
                    conti_ars408_obj_3_extended_obj_orientation_angle_decode(obj_3_extended_.obj_orientation_angle);
                }
                if (conti_ars408_obj_3_extended_obj_length_is_in_range(obj_3_extended_.obj_length)){
                    extended_temp.Length = 
                    conti_ars408_obj_3_extended_obj_length_decode(obj_3_extended_.obj_length);
                }
                if (conti_ars408_obj_3_extended_obj_width_is_in_range(obj_3_extended_.obj_width)){
                    extended_temp.Width = 
                    conti_ars408_obj_3_extended_obj_width_decode(obj_3_extended_.obj_width);
                }
                can_msg_.objs_extended.push_back(extended_temp);
                radar_60D_update_ = true;
            }
            break;
    }
    // publish when frame 32 and 35 both update at least once
    if (radar_60A_update_ && radar_60D_update_ && can_msg_.objs_extended.size() == can_msg_.objs_general.size()){
        radar_60A_update_ = false;
        radar_60D_update_ = false;
        can_msg_.header.stamp = ros::Time::now();
        can_msg_.header.frame_id = param_.radar_model;
        can_fbk_.publish(can_msg_);
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
