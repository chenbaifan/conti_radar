#include "conti_ars408_controller.h"
#include <radar_driver/Radar_Target.h>
#include <radar_driver/Radar_State_Cfg.h>

#include "can_interface/kvaser_interface.h"

namespace radar_driver {
ContiController::ContiController(const CANCardParameter &param) { param_ = param; }

void ContiController::init(ros::NodeHandle &nh) {
    is_running_ = true;
    can_reader_.reset(new radar_driver::KvaserInterface());
    can_writer_.reset(new radar_driver::KvaserInterface());
    can_reader_->Init(param_);
    can_reader_->Start();
    can_writer_->Init(param_);
    can_writer_->Start();
    radar_target_start_update_ = false;
    radar_target_end_update_ = false;
    radar_state_update_ = false;    
    
    can_target_ = nh.advertise<radar_driver::Radar_Target>("driver/radar/conti/radar_target",1);
    can_radar_state_ = nh.advertise<radar_driver::Radar_State_Cfg>("/driver/radar/conti/radar_state",1);
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
        // Frequency of different signal  
        // ID 1792 and 513 are 1HZ

        // Decode cluster information 
        // CAN frame 600 (1536) : cluster status information 
        case 1536:
            DecodeClusterStatus(frame);
            radar_target_start_update_ = true;
            break;

        // CAN frame 701 (1793) : cluster general information 
        case 1793:
            DecodeClusterGeneral(frame);
            break;

        // CAN frame 702 (1794) : cluster quality information 
        case 1794:
            DecodeClusterQuality(frame);
            radar_target_end_update_ = true;
            break;


        // Decode object information 
        // CAN frame id 60A (1546) : object list status 
        case 1546:
            DecodeObjectStatus(frame);
            radar_target_start_update_ = true;
            break;

        // CAN frame id 60B (1547) : object general information 
        case 1547:
            DecodeObjectGeneral(frame);
            break;

        // CAN frame id 60C (1548) : object quality information 
        case 1548:
            DecodeObjectQuality(frame);
            break;
        
        // CAN frame id 60D (1549) : object extended information
        case 1549:
            DecodeObjectExtended(frame);
            radar_target_end_update_ = true;
            break;


        // Radar state output 
        // CAN frame 201 (513) : radar state output 
        case 513:
            DecodeRadarState(frame);
            radar_state_update_ = true;
            break;

        // Filter state output 
        // CAN frame 203 (515) : radar filter cfg state header 
        case 515:
            DecodeFilterCfgHeader(frame);
            break;
        
        // Radar state output 
        // CAN frame 204 (516) : radar filter cfg state 
        case 516:
            DecodeFilterCfg(frame);
            break;
    }
    // publish when frame 32 and 35 both update at least once
    if (radar_target_start_update_ && radar_target_end_update_ && can_msg_radar_target_.objs_extended.size() == can_msg_radar_target_.objs_general.size()){
        radar_target_start_update_ = false;
        radar_target_end_update_ = false;
        can_msg_radar_target_.header.stamp = ros::Time::now();
        can_msg_radar_target_.header.frame_id = param_.radar_model;
        can_target_.publish(can_msg_radar_target_);
    }
    if (radar_state_update_){
        radar_state_update_ = false;
        can_msg_radar_state_cfg.header.stamp = ros::Time::now();
        can_msg_radar_state_cfg.header.frame_id = param_.radar_model;
        can_radar_state_.publish(can_msg_radar_state_cfg);
    }
    
}



void ContiController::DecodeObjectStatus(const CanFrame &frame){
    conti_ars408_obj_0_status_unpack(&obj_0_status_, frame.data, sizeof(frame.data));    
    if (conti_ars408_obj_0_status_obj_nof_objects_is_in_range(obj_0_status_.obj_nof_objects)){
        can_msg_radar_target_.objs_status.NofObjects = 
        conti_ars408_obj_0_status_obj_nof_objects_decode(obj_0_status_.obj_nof_objects);
    }
    if (conti_ars408_obj_0_status_obj_meas_counter_is_in_range(obj_0_status_.obj_meas_counter)){
        can_msg_radar_target_.objs_status.MeasCounter = 
        conti_ars408_obj_0_status_obj_meas_counter_decode(obj_0_status_.obj_meas_counter);
    }
    if (conti_ars408_obj_0_status_obj_interface_version_is_in_range(obj_0_status_.obj_interface_version)){
        can_msg_radar_target_.objs_status.InterfaceVersion = 
        conti_ars408_obj_0_status_obj_interface_version_decode(obj_0_status_.obj_interface_version);
    }
    can_msg_radar_target_.objs_general.resize(0);
    can_msg_radar_target_.objs_quality.resize(0);
    can_msg_radar_target_.objs_extended.resize(0);
}

void ContiController::DecodeObjectGeneral(const CanFrame &frame){
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
    can_msg_radar_target_.objs_general.push_back(obj_temp);
    //std::cout << "Object general information: " << std::to_string(can_msg_radar_target_.objs_general.back().DistLong) << std::endl;
}

void ContiController::DecodeObjectQuality(const CanFrame &frame){
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
    can_msg_radar_target_.objs_quality.push_back(quality_temp);
}

void ContiController::DecodeObjectExtended(const CanFrame &frame){
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
    can_msg_radar_target_.objs_extended.push_back(extended_temp);
}

void ContiController::DecodeClusterStatus(const CanFrame &frame){
    conti_ars408_cluster_0_status_unpack(&cluster_0_status_, frame.data, sizeof(frame.data));
    if (conti_ars408_cluster_0_status_cluster_nof_clusters_near_is_in_range(cluster_0_status_.cluster_nof_clusters_near)){
        can_msg_radar_target_.cluster_status.NofClusterNear = 
        conti_ars408_cluster_0_status_cluster_nof_clusters_near_decode(cluster_0_status_.cluster_nof_clusters_near);
    }
    if (conti_ars408_cluster_0_status_cluster_nof_clusters_far_is_in_range(cluster_0_status_.cluster_nof_clusters_far)){
        can_msg_radar_target_.cluster_status.NofClusterFar = 
        conti_ars408_cluster_0_status_cluster_nof_clusters_far_decode(cluster_0_status_.cluster_nof_clusters_far);
    }
    if (conti_ars408_cluster_0_status_cluster_meas_counter_is_in_range(cluster_0_status_.cluster_meas_counter)){
        can_msg_radar_target_.cluster_status.MeasCounter = 
        conti_ars408_cluster_0_status_cluster_meas_counter_decode(cluster_0_status_.cluster_meas_counter);
    }
    if (conti_ars408_cluster_0_status_cluster_interface_version_is_in_range(cluster_0_status_.cluster_interface_version)){
        can_msg_radar_target_.cluster_status.InterfaceVersion = 
        conti_ars408_cluster_0_status_cluster_interface_version_decode(cluster_0_status_.cluster_interface_version);
    }
}

void ContiController::DecodeClusterGeneral(const CanFrame &frame){
    conti_ars408_cluster_1_general_unpack(&cluster_1_general_, frame.data, sizeof(frame.data));
    radar_driver::Conti_cluster_general general_temp;
    if (conti_ars408_cluster_1_general_cluster_id_is_in_range(cluster_1_general_.cluster_id)){
        general_temp.ID = conti_ars408_cluster_1_general_cluster_id_decode(cluster_1_general_.cluster_id);
    }
    if (conti_ars408_cluster_1_general_cluster_dist_long_is_in_range(cluster_1_general_.cluster_dist_long)){
        general_temp.DistLong = conti_ars408_cluster_1_general_cluster_dist_long_decode(cluster_1_general_.cluster_dist_long);
    }
    if (conti_ars408_cluster_1_general_cluster_dist_lat_is_in_range(cluster_1_general_.cluster_dist_lat)){
        general_temp.DistLat = conti_ars408_cluster_1_general_cluster_dist_lat_decode(cluster_1_general_.cluster_dist_lat);
    }
    if (conti_ars408_cluster_1_general_cluster_vrel_long_is_in_range(cluster_1_general_.cluster_vrel_long)){
        general_temp.VrelLong = conti_ars408_cluster_1_general_cluster_vrel_long_decode(cluster_1_general_.cluster_vrel_long);
    }
    if (conti_ars408_cluster_1_general_cluster_dyn_prop_is_in_range(cluster_1_general_.cluster_dyn_prop)){
        general_temp.DynProp = conti_ars408_cluster_1_general_cluster_dyn_prop_decode(cluster_1_general_.cluster_dyn_prop);
    }
    if (conti_ars408_cluster_1_general_cluster_vrel_lat_is_in_range(cluster_1_general_.cluster_vrel_lat)){
        general_temp.VrelLat = conti_ars408_cluster_1_general_cluster_vrel_lat_decode(cluster_1_general_.cluster_vrel_lat);
    }
    if (conti_ars408_cluster_1_general_cluster_rcs_is_in_range(cluster_1_general_.cluster_rcs)){
        general_temp.RCS = conti_ars408_cluster_1_general_cluster_rcs_decode(cluster_1_general_.cluster_rcs);
    }
    can_msg_radar_target_.cluster_general.push_back(general_temp);
}

void ContiController::DecodeClusterQuality(const CanFrame &frame){
    conti_ars408_cluster_2_quality_unpack(&cluster_2_quality_, frame.data, sizeof(frame.data));
    radar_driver::Conti_cluster_quality quality_temp;
    if (conti_ars408_cluster_2_quality_cluster_id_is_in_range(cluster_2_quality_.cluster_id)){
        quality_temp.ID = conti_ars408_cluster_2_quality_cluster_id_decode(cluster_2_quality_.cluster_id);
    }
    if (conti_ars408_cluster_2_quality_cluster_dist_long_rms_is_in_range(cluster_2_quality_.cluster_dist_long_rms)){
        quality_temp.DistLong_rms = conti_ars408_cluster_2_quality_cluster_dist_long_rms_decode(cluster_2_quality_.cluster_dist_long_rms);
    }
    if (conti_ars408_cluster_2_quality_cluster_vrel_long_rms_is_in_range(cluster_2_quality_.cluster_vrel_long_rms)){
        quality_temp.VrelLong_rms = conti_ars408_cluster_2_quality_cluster_vrel_long_rms_decode(cluster_2_quality_.cluster_vrel_long_rms);
    }
    if (conti_ars408_cluster_2_quality_cluster_dist_lat_rms_is_in_range(cluster_2_quality_.cluster_dist_lat_rms)){
        quality_temp.DisLat_rms = conti_ars408_cluster_2_quality_cluster_dist_lat_rms_decode(cluster_2_quality_.cluster_dist_lat_rms);
    }
    if (conti_ars408_cluster_2_quality_cluster_pd_h0_is_in_range(cluster_2_quality_.cluster_pd_h0)){
        quality_temp.Pdh0 = conti_ars408_cluster_2_quality_cluster_pd_h0_decode(cluster_2_quality_.cluster_pd_h0);
    }
    if (conti_ars408_cluster_2_quality_cluster_vrel_lat_rms_is_in_range(cluster_2_quality_.cluster_vrel_lat_rms)){
        quality_temp.VrelLat_rms = conti_ars408_cluster_2_quality_cluster_vrel_lat_rms_decode(cluster_2_quality_.cluster_vrel_lat_rms) ;
    }
    if (conti_ars408_cluster_2_quality_cluster_ambig_state_is_in_range(cluster_2_quality_.cluster_ambig_state)){
        quality_temp.AmbigState = conti_ars408_cluster_2_quality_cluster_ambig_state_decode(cluster_2_quality_.cluster_ambig_state);
    }
    if (conti_ars408_cluster_2_quality_cluster_invalid_state_is_in_range(cluster_2_quality_.cluster_invalid_state)){
        quality_temp.InvalidState = conti_ars408_cluster_2_quality_cluster_invalid_state_decode(cluster_2_quality_.cluster_invalid_state);
    }
    can_msg_radar_target_.cluster_quality.push_back(quality_temp);
}
    


void ContiController::DecodeRadarState(const CanFrame &frame){
    conti_ars408_radar_state_unpack(&radar_state_, frame.data, sizeof(frame.data));
    if (conti_ars408_radar_state_radar_state_nvm_read_status_is_in_range(radar_state_.radar_state_nvm_read_status)){
        can_msg_radar_state_cfg.radar_state.NVMReadStatus = 
        conti_ars408_radar_state_radar_state_nvm_read_status_decode(radar_state_.radar_state_nvm_read_status);
    }
    if (conti_ars408_radar_state_radar_state_nv_mwrite_status_is_in_range(radar_state_.radar_state_nv_mwrite_status)){
        can_msg_radar_state_cfg.radar_state.NVMwriteStatus = 
        conti_ars408_radar_state_radar_state_nv_mwrite_status_decode(radar_state_.radar_state_nv_mwrite_status);
    }
    if (conti_ars408_radar_state_radar_state_max_distance_cfg_is_in_range(radar_state_.radar_state_max_distance_cfg)){
        can_msg_radar_state_cfg.radar_state.MaxDistanceCfg = 
        conti_ars408_radar_state_radar_state_max_distance_cfg_decode(radar_state_.radar_state_max_distance_cfg);
    }
    if (conti_ars408_radar_state_radar_state_persistent_error_is_in_range(radar_state_.radar_state_persistent_error)){
        can_msg_radar_state_cfg.radar_state.Persistent_Error = 
        conti_ars408_radar_state_radar_state_persistent_error_decode(radar_state_.radar_state_persistent_error);
    }
    if (conti_ars408_radar_state_radar_state_interference_is_in_range(radar_state_.radar_state_interference)){
        can_msg_radar_state_cfg.radar_state.Persistent_Error = 
        conti_ars408_radar_state_radar_state_interference_decode(radar_state_.radar_state_interference);
    }
    if (conti_ars408_radar_state_radar_state_temperature_error_is_in_range(radar_state_.radar_state_temperature_error)){
        can_msg_radar_state_cfg.radar_state.Temperature_Error = 
        conti_ars408_radar_state_radar_state_temperature_error_decode(radar_state_.radar_state_temperature_error);
    }
    if (conti_ars408_radar_state_radar_state_temporary_error_is_in_range(radar_state_.radar_state_temporary_error)){
        can_msg_radar_state_cfg.radar_state.Temporary_Error = 
        conti_ars408_radar_state_radar_state_temporary_error_decode(radar_state_.radar_state_temporary_error);
    }
    if (conti_ars408_radar_state_radar_state_voltage_error_is_in_range(radar_state_.radar_state_voltage_error)){
        can_msg_radar_state_cfg.radar_state.Voltage_Error = 
        conti_ars408_radar_state_radar_state_temporary_error_decode(radar_state_.radar_state_voltage_error);
    }
    if (conti_ars408_radar_state_radar_state_sensor_id_is_in_range(radar_state_.radar_state_sensor_id)){
        can_msg_radar_state_cfg.radar_state.SensorID = 
        conti_ars408_radar_state_radar_state_sensor_id_decode(radar_state_.radar_state_sensor_id);
    }
    if (conti_ars408_radar_state_radar_state_sort_index_is_in_range(radar_state_.radar_state_sort_index)){
        can_msg_radar_state_cfg.radar_state.SortIndex = 
        conti_ars408_radar_state_radar_state_sort_index_decode(radar_state_.radar_state_sort_index);
    }
    if (conti_ars408_radar_state_radar_state_radar_power_cfg_is_in_range(radar_state_.radar_state_radar_power_cfg)){
        can_msg_radar_state_cfg.radar_state.RadarPowerCfg = 
        conti_ars408_radar_state_radar_state_radar_power_cfg_decode(radar_state_.radar_state_radar_power_cfg);
    }
    if (conti_ars408_radar_state_radar_state_ctrl_relay_cfg_is_in_range(radar_state_.radar_state_ctrl_relay_cfg)){
        can_msg_radar_state_cfg.radar_state.CtrlRelayCfg = 
        conti_ars408_radar_state_radar_state_ctrl_relay_cfg_decode(radar_state_.radar_state_ctrl_relay_cfg);
    }
    if (conti_ars408_radar_state_radar_state_output_type_cfg_is_in_range(radar_state_.radar_state_output_type_cfg)){
        can_msg_radar_state_cfg.radar_state.OutputTypeCfg = 
        conti_ars408_radar_state_radar_state_output_type_cfg_decode(radar_state_.radar_state_output_type_cfg);
    }
    if (conti_ars408_radar_state_radar_state_send_quality_cfg_is_in_range(radar_state_.radar_state_send_quality_cfg)){
        can_msg_radar_state_cfg.radar_state.SendQualityCfg = 
        conti_ars408_radar_state_radar_state_send_quality_cfg_decode(radar_state_.radar_state_send_quality_cfg);
    }
    if (conti_ars408_radar_state_radar_state_send_ext_info_cfg_is_in_range(radar_state_.radar_state_send_ext_info_cfg)){
        can_msg_radar_state_cfg.radar_state.SendExtInfoCfg = 
        conti_ars408_radar_state_radar_state_send_ext_info_cfg_decode(radar_state_.radar_state_send_ext_info_cfg);
    }
    if (conti_ars408_radar_state_radar_state_motion_rx_state_is_in_range(radar_state_.radar_state_motion_rx_state)){
        can_msg_radar_state_cfg.radar_state.MotionRxState = 
        conti_ars408_radar_state_radar_state_motion_rx_state_decode(radar_state_.radar_state_motion_rx_state);
    }
    if (conti_ars408_radar_state_radar_state_rcs_threshold_is_in_range(radar_state_.radar_state_rcs_threshold)){
        can_msg_radar_state_cfg.radar_state.RCS_Threshold = 
        conti_ars408_radar_state_radar_state_rcs_threshold_decode(radar_state_.radar_state_rcs_threshold);
    }
}

void ContiController::DecodeFilterCfgHeader(const CanFrame &frame){
    conti_ars408_filter_state_header_unpack(&filter_state_header_, frame.data, sizeof(frame.data));
    if (conti_ars408_filter_state_header_filter_state_nof_cluster_filter_cfg_is_in_range(filter_state_header_.filter_state_nof_cluster_filter_cfg)){
        can_msg_radar_state_cfg.filter_config.Nof_Cluster_FilterCfg = 
        conti_ars408_filter_state_header_filter_state_nof_cluster_filter_cfg_decode(filter_state_header_.filter_state_nof_cluster_filter_cfg);
    }
    if (conti_ars408_filter_state_header_filter_state_nof_object_filter_cfg_is_in_range(filter_state_header_.filter_state_nof_object_filter_cfg)){
        can_msg_radar_state_cfg.filter_config.Nof_Object_FilterCfg = 
        conti_ars408_filter_state_header_filter_state_nof_object_filter_cfg_decode(filter_state_header_.filter_state_nof_object_filter_cfg);
    }
}

void ContiController::DecodeFilterCfg(const CanFrame &frame){
    conti_ars408_filter_cfg_unpack(&filter_cfg_, frame.data, sizeof(frame.data));
    if (conti_ars408_filter_state_cfg_filter_state_active_is_in_range(filter_cfg_.filter_cfg_active)){
        can_msg_radar_state_cfg.filter_config.Active = 
        conti_ars408_filter_state_cfg_filter_state_active_decode(filter_cfg_.filter_cfg_active);
    }
    if (conti_ars408_filter_state_cfg_filter_state_index_is_in_range(filter_cfg_.filter_cfg_index)){
        can_msg_radar_state_cfg.filter_config.Index = 
        conti_ars408_filter_state_cfg_filter_state_index_decode(filter_cfg_.filter_cfg_index);
    }
    if (conti_ars408_filter_state_cfg_filter_state_type_is_in_range(filter_cfg_.filter_cfg_type)){
        can_msg_radar_state_cfg.filter_config.Output_Type = 
        conti_ars408_filter_state_cfg_filter_state_type_decode(filter_cfg_.filter_cfg_type);
    }
    // MIN
    if (conti_ars408_filter_state_cfg_filter_state_min_nof_obj_is_in_range(filter_cfg_.filter_cfg_min_nof_obj)){
        can_msg_radar_state_cfg.filter_config.Min_NofObj = 
        conti_ars408_filter_state_cfg_filter_state_min_nof_obj_decode(filter_cfg_.filter_cfg_min_nof_obj);
    }
    if (conti_ars408_filter_state_cfg_filter_state_min_distance_is_in_range(filter_cfg_.filter_cfg_min_distance)){
        can_msg_radar_state_cfg.filter_config.Min_Distance = 
        conti_ars408_filter_state_cfg_filter_state_min_distance_decode(filter_cfg_.filter_cfg_min_distance);
    }
    if (conti_ars408_filter_state_cfg_filter_state_min_azimuth_is_in_range(filter_cfg_.filter_cfg_min_azimuth)){
        can_msg_radar_state_cfg.filter_config.Min_Azimuth = 
        conti_ars408_filter_state_cfg_filter_state_min_azimuth_decode(filter_cfg_.filter_cfg_min_azimuth);
    }
    if (conti_ars408_filter_state_cfg_filter_state_min_vrel_oncome_is_in_range(filter_cfg_.filter_cfg_min_vrel_oncome)){
        can_msg_radar_state_cfg.filter_config.Min_VrelOncome = 
        conti_ars408_filter_state_cfg_filter_state_min_vrel_oncome_decode(filter_cfg_.filter_cfg_min_vrel_oncome);
    }
    if (conti_ars408_filter_state_cfg_filter_state_min_vrel_depart_is_in_range(filter_cfg_.filter_cfg_min_vrel_depart)){
        can_msg_radar_state_cfg.filter_config.Min_VrelDepart = 
        conti_ars408_filter_state_cfg_filter_state_min_vrel_depart_decode(filter_cfg_.filter_cfg_min_vrel_depart);
    }
    if (conti_ars408_filter_state_cfg_filter_state_min_rcs_is_in_range(filter_cfg_.filter_cfg_min_rcs)){
        can_msg_radar_state_cfg.filter_config.Min_RCS = 
        conti_ars408_filter_state_cfg_filter_state_min_rcs_decode(filter_cfg_.filter_cfg_min_rcs);
    }
    if (conti_ars408_filter_state_cfg_filter_state_min_lifetime_is_in_range(filter_cfg_.filter_cfg_min_lifetime)){
        can_msg_radar_state_cfg.filter_config.Min_Lifetime = 
        conti_ars408_filter_state_cfg_filter_state_min_lifetime_decode(filter_cfg_.filter_cfg_min_lifetime);
    }
    if (conti_ars408_filter_state_cfg_filter_state_min_size_is_in_range(filter_cfg_.filter_cfg_min_size)){
        can_msg_radar_state_cfg.filter_config.Min_Size = 
        conti_ars408_filter_state_cfg_filter_state_min_size_decode(filter_cfg_.filter_cfg_min_size);
    }
    if (conti_ars408_filter_state_cfg_filter_state_min_prob_exists_is_in_range(filter_cfg_.filter_cfg_min_prob_exists)){
        can_msg_radar_state_cfg.filter_config.Min_ProbExists = 
        conti_ars408_filter_state_cfg_filter_state_min_prob_exists_decode(filter_cfg_.filter_cfg_min_prob_exists);
    }
    if (conti_ars408_filter_state_cfg_filter_state_min_y_is_in_range(filter_cfg_.filter_cfg_min_y)){
        can_msg_radar_state_cfg.filter_config.Min_Y = 
        conti_ars408_filter_state_cfg_filter_state_min_y_decode(filter_cfg_.filter_cfg_min_y);
    }
    if (conti_ars408_filter_state_cfg_filter_state_min_x_is_in_range(filter_cfg_.filter_cfg_min_x)){
        can_msg_radar_state_cfg.filter_config.Min_X = 
        conti_ars408_filter_state_cfg_filter_state_min_x_decode(filter_cfg_.filter_cfg_min_x);
    }
    if (conti_ars408_filter_state_cfg_filter_state_min_vy_left_right_is_in_range(filter_cfg_.filter_cfg_max_vy_left_right)){
        can_msg_radar_state_cfg.filter_config.Min_VYRightLeft = 
        conti_ars408_filter_state_cfg_filter_state_min_vy_left_right_decode(filter_cfg_.filter_cfg_min_vy_left_right);
    }
    if (conti_ars408_filter_state_cfg_filter_state_min_vx_oncome_is_in_range(filter_cfg_.filter_cfg_min_vx_oncome)){
        can_msg_radar_state_cfg.filter_config.Min_VXOncome = 
        conti_ars408_filter_state_cfg_filter_state_min_vx_oncome_decode(filter_cfg_.filter_cfg_min_vx_oncome);
    }
    if (conti_ars408_filter_state_cfg_filter_state_min_vy_right_left_is_in_range(filter_cfg_.filter_cfg_min_vy_right_left)){
        can_msg_radar_state_cfg.filter_config.Min_VYRightLeft = 
        conti_ars408_filter_state_cfg_filter_state_min_vy_right_left_decode(filter_cfg_.filter_cfg_min_vy_right_left); 
    }
    if (conti_ars408_filter_state_cfg_filter_state_min_vx_depart_is_in_range(filter_cfg_.filter_cfg_min_vx_depart)){
        can_msg_radar_state_cfg.filter_config.Min_VXDepart = 
        conti_ars408_filter_state_cfg_filter_state_min_vx_depart_decode(filter_cfg_.filter_cfg_min_vx_depart);
    }
    // MAX
    if (conti_ars408_filter_state_cfg_filter_state_max_nof_obj_is_in_range(filter_cfg_.filter_cfg_max_nof_obj)){
        can_msg_radar_state_cfg.filter_config.Max_NofObj = 
        conti_ars408_filter_state_cfg_filter_state_max_nof_obj_decode(filter_cfg_.filter_cfg_max_nof_obj);
    }
    if (conti_ars408_filter_state_cfg_filter_state_max_distance_is_in_range(filter_cfg_.filter_cfg_max_distance)){
        can_msg_radar_state_cfg.filter_config.Max_Distance = 
        conti_ars408_filter_state_cfg_filter_state_max_distance_decode(filter_cfg_.filter_cfg_max_distance);
    }
    if (conti_ars408_filter_state_cfg_filter_state_max_azimuth_is_in_range(filter_cfg_.filter_cfg_max_azimuth)){
        can_msg_radar_state_cfg.filter_config.Max_Azimuth = 
        conti_ars408_filter_state_cfg_filter_state_max_azimuth_decode(filter_cfg_.filter_cfg_max_azimuth);
    }
    if (conti_ars408_filter_state_cfg_filter_state_max_vrel_oncome_is_in_range(filter_cfg_.filter_cfg_max_vrel_oncome)){
        can_msg_radar_state_cfg.filter_config.Max_VrelOncome = 
        conti_ars408_filter_state_cfg_filter_state_max_vrel_oncome_decode(filter_cfg_.filter_cfg_max_vrel_oncome);
    }
    if (conti_ars408_filter_state_cfg_filter_state_max_vrel_depart_is_in_range(filter_cfg_.filter_cfg_max_vrel_depart)){
        can_msg_radar_state_cfg.filter_config.Max_VrelDepart = 
        conti_ars408_filter_state_cfg_filter_state_max_vrel_depart_decode(filter_cfg_.filter_cfg_max_vrel_depart);
    }
    if (conti_ars408_filter_state_cfg_filter_state_max_rcs_is_in_range(filter_cfg_.filter_cfg_max_rcs)){
        can_msg_radar_state_cfg.filter_config.Max_RCS = 
        conti_ars408_filter_state_cfg_filter_state_max_rcs_decode(filter_cfg_.filter_cfg_max_rcs);
    }
    if (conti_ars408_filter_state_cfg_filter_state_max_lifetime_is_in_range(filter_cfg_.filter_cfg_max_lifetime)){
        can_msg_radar_state_cfg.filter_config.Max_Lifetime = 
        conti_ars408_filter_state_cfg_filter_state_max_lifetime_decode(filter_cfg_.filter_cfg_max_lifetime);
    }
    if (conti_ars408_filter_state_cfg_filter_state_max_size_is_in_range(filter_cfg_.filter_cfg_max_size)){
        can_msg_radar_state_cfg.filter_config.Max_Size = 
        conti_ars408_filter_state_cfg_filter_state_max_size_decode(filter_cfg_.filter_cfg_max_size);
    }
    if (conti_ars408_filter_state_cfg_filter_state_max_prob_exists_is_in_range(filter_cfg_.filter_cfg_max_prob_exists)){
        can_msg_radar_state_cfg.filter_config.Max_ProbExists = 
        conti_ars408_filter_state_cfg_filter_state_max_prob_exists_decode(filter_cfg_.filter_cfg_max_prob_exists);
    }
    if (conti_ars408_filter_state_cfg_filter_state_max_y_is_in_range(filter_cfg_.filter_cfg_max_y)){
        can_msg_radar_state_cfg.filter_config.Max_Y = 
        conti_ars408_filter_state_cfg_filter_state_max_y_decode(filter_cfg_.filter_cfg_max_y);
    }
    if (conti_ars408_filter_state_cfg_filter_state_max_x_is_in_range(filter_cfg_.filter_cfg_max_x)){
        can_msg_radar_state_cfg.filter_config.Max_X = 
        conti_ars408_filter_state_cfg_filter_state_max_x_decode(filter_cfg_.filter_cfg_max_x);
    }
    if (conti_ars408_filter_state_cfg_filter_state_max_vy_left_right_is_in_range(filter_cfg_.filter_cfg_max_vy_left_right)){
        can_msg_radar_state_cfg.filter_config.Max_VYRightLeft = 
        conti_ars408_filter_state_cfg_filter_state_max_vy_left_right_decode(filter_cfg_.filter_cfg_max_vy_left_right);
    }
    if (conti_ars408_filter_state_cfg_filter_state_max_vx_oncome_is_in_range(filter_cfg_.filter_cfg_max_vx_oncome)){
        can_msg_radar_state_cfg.filter_config.Max_VXOncome = 
        conti_ars408_filter_state_cfg_filter_state_max_vx_oncome_decode(filter_cfg_.filter_cfg_max_vx_oncome);
    }
    if (conti_ars408_filter_state_cfg_filter_state_max_vy_right_left_is_in_range(filter_cfg_.filter_cfg_max_vy_right_left)){
        can_msg_radar_state_cfg.filter_config.Max_VYRightLeft = 
        conti_ars408_filter_state_cfg_filter_state_max_vy_right_left_decode(filter_cfg_.filter_cfg_max_vy_right_left); 
    }
    if (conti_ars408_filter_state_cfg_filter_state_max_vx_depart_is_in_range(filter_cfg_.filter_cfg_max_vx_depart)){
        can_msg_radar_state_cfg.filter_config.Max_VXDepart = 
        conti_ars408_filter_state_cfg_filter_state_max_vx_depart_decode(filter_cfg_.filter_cfg_max_vx_depart);
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
