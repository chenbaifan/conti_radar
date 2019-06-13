#include "conti_ars408_controller.h"
#include <radar_driver/Radar_Target.h>
#include <radar_driver/Radar_State_Cfg.h>
#include <pb_msgs/utils.h>

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
    speed_update_ = false;
    yawrate_update_ = false;
    radar_quality_enable_ = true;
    radar_extended_enable_ = true;
    radar_output_mode_ = 1;
    num_of_target_ = -1;
    // Decode can_msg to ros_msg
    can_radar_trackarray_ = nh.advertise<radar_driver::RadarTrackArray>("/driver/radar/conti/radar_target",1);
    can_radar_state_ = nh.advertise<radar_driver::Radar_State_Cfg>("/driver/radar/conti/radar_state",1);
    // Encode ros_msg to can_msg
    can_cmd_radar_cfg_ = nh.subscribe("/Perception/radar/radar_cfg", 1,
                            &ContiController::EncodeMsgCallback_RadarCfg, this);
    can_cmd_motion_ = nh.subscribe("/vehicle/canFeedback", 100,
                            &ContiController::EncodeMsgCallback_Motion, this);
    can_cmd_yawrate_ = nh.subscribe("/polyx_correctedIMU", 100,
                            &ContiController::EncodeMsgCallback_Yawrate, this);

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
            if (radar_target_start_update_){
                DecodeClusterGeneral(frame);
            }
            break;

        // CAN frame 702 (1794) : cluster quality information 
        case 1794:
            if (radar_target_start_update_){
                DecodeClusterQuality(frame);
            }
            break;

        // Decode object information 
        // CAN frame id 60A (1546) : object list status 
        case 1546:
            DecodeObjectStatus(frame);
            radar_target_start_update_ = true;
            break;

        // CAN frame id 60B (1547) : object general information 
        case 1547:
            if (radar_target_start_update_){
                DecodeObjectGeneral(frame);
            }
            break;

        // CAN frame id 60C (1548) : object quality information 
        case 1548:
            if (radar_target_start_update_){
                DecodeObjectQuality(frame);
            }
            break;
        
        // CAN frame id 60D (1549) : object extended information
        case 1549:
            if (radar_target_start_update_){
                DecodeObjectExtended(frame);
            }
            // radar_target_end_update_ = true;
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
    bool flag1 = false, flag2 = false;
    if (radar_target_start_update_){
        if (radar_output_mode_ == 1){
            flag1 = radar_quality_enable_ ? (radar_target_.objs_quality.size() == num_of_target_) : (radar_target_.objs_general.size() == num_of_target_); 
            flag2 = radar_extended_enable_ ? (radar_target_.objs_extended.size() == num_of_target_) : flag1;
        }else if(radar_output_mode_ == 2){
            flag2 = radar_quality_enable_ ? (radar_target_.cluster_quality.size() == num_of_target_) : (radar_target_.cluster_general.size() == num_of_target_);
        }
        if (flag2){
            if (radar_quality_enable_) std::cout<<"Output with quality information " << std::endl;
            if (radar_extended_enable_) std::cout << "Output with extended information " << std::endl;
            radar_target_start_update_ = false;
            can_msg_radar_track_array.tracks.resize(0);
            ContiController::Combine2TrackArray();
            can_msg_radar_track_array.header.stamp = ros::Time::now();
            can_msg_radar_track_array.header.frame_id = "radar_conti";
            can_radar_trackarray_.publish(can_msg_radar_track_array);
        }
    }
    if (radar_state_update_){
        radar_state_update_ = false;
        radar_quality_enable_ = can_msg_radar_state_cfg.radar_state.SendQualityCfg == 1 ? true : false;
        radar_extended_enable_ = can_msg_radar_state_cfg.radar_state.SendExtInfoCfg == 1 ? true : false;
        radar_output_mode_ = can_msg_radar_state_cfg.radar_state.OutputTypeCfg;
        can_msg_radar_state_cfg.header.stamp = ros::Time::now();
        can_msg_radar_state_cfg.header.frame_id = "radar_conti";
        can_radar_state_.publish(can_msg_radar_state_cfg);
    }
}

void ContiController::Combine2TrackArray(){
    
    std::cout << "\nobj_size : " << radar_target_.objs_general.size()<<std::endl;
    std::cout << "cluster_size : " << radar_target_.cluster_general.size()<<std::endl;
    if (radar_target_.objs_general.size() == 0 && radar_target_.cluster_general.size() > 0)
    {
        int i = 0;
        for ( i ; i < radar_target_.cluster_general.size() && 
                radar_target_.cluster_general[i].ID == radar_target_.cluster_quality[i].ID; i++){
            radar_driver::RadarTrack track_temp;
            track_temp.cluster_nof_far = radar_target_.cluster_status.NofClusterFar;
            track_temp.cluster_nof_near = radar_target_.cluster_status.NofClusterNear;
            track_temp.meas_counter = radar_target_.cluster_status.MeasCounter;
            track_temp.track_dist_long = radar_target_.cluster_general[i].DistLong;
            track_temp.track_dist_lat = radar_target_.cluster_general[i].DistLat;
            track_temp.track_vrel_long = radar_target_.cluster_general[i].VrelLong;
            track_temp.track_vrel_lat = radar_target_.cluster_general[i].VrelLat;
            track_temp.track_dyn_prop = radar_target_.cluster_general[i].DynProp;
            track_temp.track_RCS = radar_target_.cluster_general[i].RCS;
            if (radar_quality_enable_){
                track_temp.track_dist_long_rms = radar_target_.cluster_quality[i].DistLong_rms;
                track_temp.track_dist_lat_rms = radar_target_.cluster_quality[i].DistLat_rms;
                track_temp.track_vrel_long_rms = radar_target_.cluster_quality[i].VrelLong_rms;
                track_temp.track_vrel_lat_rms = radar_target_.cluster_quality[i].VrelLat_rms;
                track_temp.cluster_phd0 = radar_target_.cluster_quality[i].Pdh0;
                track_temp.cluster_abvig_state = radar_target_.cluster_quality[i].AmbigState;
                track_temp.cluster_ivalid_state = radar_target_.cluster_quality[i].InvalidState;
            }
            can_msg_radar_track_array.tracks.push_back(track_temp);
        }

    }
    else if (radar_target_.objs_general.size() > 0 && radar_target_.cluster_general.size() == 0)
    {
        int i = 0;
        for ( i ; i < radar_target_.objs_general.size() && 
                radar_target_.objs_general[i].ID == radar_target_.objs_general[i].ID; i++){
            radar_driver::RadarTrack track_temp;
            track_temp.nof_objects = radar_target_.objs_status.NofObjects;
            track_temp.meas_counter = radar_target_.objs_status.MeasCounter;
            track_temp.track_dist_long = radar_target_.objs_general[i].DistLong;
            track_temp.track_dist_lat = radar_target_.objs_general[i].DistLat;
            track_temp.track_vrel_long = radar_target_.objs_general[i].VrelLong;
            track_temp.track_vrel_lat = radar_target_.objs_general[i].VrelLat;
            track_temp.track_dyn_prop = radar_target_.objs_general[i].DynProp;
            track_temp.track_RCS = radar_target_.objs_general[i].RCS;
            if (radar_quality_enable_){
                track_temp.track_dist_long_rms = radar_target_.objs_quality[i].DistLong_rms;
                track_temp.track_dist_lat_rms = radar_target_.objs_quality[i].DistLat_rms;
                track_temp.track_vrel_long_rms = radar_target_.objs_quality[i].VrelLong_rms;
                track_temp.track_vrel_lat_rms = radar_target_.objs_quality[i].VrelLat_rms;
                track_temp.track_arel_long_rms = radar_target_.objs_quality[i].ArelLong_rms;
                track_temp.track_arel_lat_rms = radar_target_.objs_quality[i].ArelLat_rms;
                track_temp.track_orientation_rms = radar_target_.objs_quality[i].Orientation_rms;
                track_temp.track_meas_state = radar_target_.objs_quality[i].MeasState;
                track_temp.track_prob_ofexist = radar_target_.objs_quality[i].ProbOfExist;
            }
            if (radar_extended_enable_){
                track_temp.track_arel_long = radar_target_.objs_extended[i].ArelLong;
                track_temp.track_arel_lat = radar_target_.objs_extended[i].ArelLat;
                track_temp.track_class = radar_target_.objs_extended[i].Class;
                track_temp.track_orientation_angle = radar_target_.objs_extended[i].OrientationAngle;
                track_temp.track_length = radar_target_.objs_extended[i].Length;
                track_temp.track_width = radar_target_.objs_extended[i].Width;
            }
            can_msg_radar_track_array.tracks.push_back(track_temp);
        }
    }
    else if (radar_target_.objs_general.size() == 0 && radar_target_.cluster_general.size() == 0){
            return;
    }
    else {
            ROS_ERROR("Have output from both cluster and objs");
    }
}


void ContiController::DecodeObjectStatus(const CanFrame &frame){
    conti_ars408_obj_0_status_unpack(&obj_0_status_, frame.data, sizeof(frame.data));    
    if (conti_ars408_obj_0_status_obj_nof_objects_is_in_range(obj_0_status_.obj_nof_objects)){
        radar_target_.objs_status.NofObjects = 
        conti_ars408_obj_0_status_obj_nof_objects_decode(obj_0_status_.obj_nof_objects);
    }
    if (conti_ars408_obj_0_status_obj_meas_counter_is_in_range(obj_0_status_.obj_meas_counter)){
        radar_target_.objs_status.MeasCounter = 
        conti_ars408_obj_0_status_obj_meas_counter_decode(obj_0_status_.obj_meas_counter);
    }
    if (conti_ars408_obj_0_status_obj_interface_version_is_in_range(obj_0_status_.obj_interface_version)){
        radar_target_.objs_status.InterfaceVersion = 
        conti_ars408_obj_0_status_obj_interface_version_decode(obj_0_status_.obj_interface_version);
    }
    num_of_target_ = radar_target_.objs_status.NofObjects;
    radar_driver::Radar_Target tmp;
    radar_target_ = tmp;
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
    radar_target_.objs_general.push_back(obj_temp);
    //std::cout << "Object general information: " << std::to_string(radar_target_.objs_general.back().DistLong) << std::endl;
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
        quality_temp.DistLat_rms = 
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
    radar_target_.objs_quality.push_back(quality_temp);
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
    radar_target_.objs_extended.push_back(extended_temp);
}

void ContiController::DecodeClusterStatus(const CanFrame &frame){
    conti_ars408_cluster_0_status_unpack(&cluster_0_status_, frame.data, sizeof(frame.data));
    if (conti_ars408_cluster_0_status_cluster_nof_clusters_near_is_in_range(cluster_0_status_.cluster_nof_clusters_near)){
        radar_target_.cluster_status.NofClusterNear = 
        conti_ars408_cluster_0_status_cluster_nof_clusters_near_decode(cluster_0_status_.cluster_nof_clusters_near);
    }
    if (conti_ars408_cluster_0_status_cluster_nof_clusters_far_is_in_range(cluster_0_status_.cluster_nof_clusters_far)){
        radar_target_.cluster_status.NofClusterFar = 
        conti_ars408_cluster_0_status_cluster_nof_clusters_far_decode(cluster_0_status_.cluster_nof_clusters_far);
    }
    if (conti_ars408_cluster_0_status_cluster_meas_counter_is_in_range(cluster_0_status_.cluster_meas_counter)){
        radar_target_.cluster_status.MeasCounter = 
        conti_ars408_cluster_0_status_cluster_meas_counter_decode(cluster_0_status_.cluster_meas_counter);
    }
    if (conti_ars408_cluster_0_status_cluster_interface_version_is_in_range(cluster_0_status_.cluster_interface_version)){
        radar_target_.cluster_status.InterfaceVersion = 
        conti_ars408_cluster_0_status_cluster_interface_version_decode(cluster_0_status_.cluster_interface_version);
    }
    num_of_target_ = radar_target_.cluster_status.NofClusterNear + radar_target_.cluster_status.NofClusterFar;
    radar_driver::Radar_Target tmp;
    radar_target_ = tmp;
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
    radar_target_.cluster_general.push_back(general_temp);
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
        quality_temp.DistLat_rms = conti_ars408_cluster_2_quality_cluster_dist_lat_rms_decode(cluster_2_quality_.cluster_dist_lat_rms);
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
    radar_target_.cluster_quality.push_back(quality_temp);
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
    conti_ars408_filter_state_cfg_unpack(&filter_state_cfg_, frame.data, sizeof(frame.data));
    if (conti_ars408_filter_state_cfg_filter_state_active_is_in_range(filter_state_cfg_.filter_state_active)){
        can_msg_radar_state_cfg.filter_config.Active = 
        conti_ars408_filter_state_cfg_filter_state_active_decode(filter_state_cfg_.filter_state_active);
    }
    if (conti_ars408_filter_state_cfg_filter_state_index_is_in_range(filter_state_cfg_.filter_state_index)){
        can_msg_radar_state_cfg.filter_config.Index = 
        conti_ars408_filter_state_cfg_filter_state_index_decode(filter_state_cfg_.filter_state_index);
    }
    if (conti_ars408_filter_state_cfg_filter_state_type_is_in_range(filter_state_cfg_.filter_state_type)){
        can_msg_radar_state_cfg.filter_config.Output_Type = 
        conti_ars408_filter_state_cfg_filter_state_type_decode(filter_state_cfg_.filter_state_type);
    }
    // MIN
    if (conti_ars408_filter_state_cfg_filter_state_min_nof_obj_is_in_range(filter_state_cfg_.filter_state_min_nof_obj)){
        can_msg_radar_state_cfg.filter_config.Min_NofObj = 
        conti_ars408_filter_state_cfg_filter_state_min_nof_obj_decode(filter_state_cfg_.filter_state_min_nof_obj);
    }
    if (conti_ars408_filter_state_cfg_filter_state_min_distance_is_in_range(filter_state_cfg_.filter_state_min_distance)){
        can_msg_radar_state_cfg.filter_config.Min_Distance = 
        conti_ars408_filter_state_cfg_filter_state_min_distance_decode(filter_state_cfg_.filter_state_min_distance);
    }
    if (conti_ars408_filter_state_cfg_filter_state_min_azimuth_is_in_range(filter_state_cfg_.filter_state_min_azimuth)){
        can_msg_radar_state_cfg.filter_config.Min_Azimuth = 
        conti_ars408_filter_state_cfg_filter_state_min_azimuth_decode(filter_state_cfg_.filter_state_min_azimuth);
    }
    if (conti_ars408_filter_state_cfg_filter_state_min_vrel_oncome_is_in_range(filter_state_cfg_.filter_state_min_vrel_oncome)){
        can_msg_radar_state_cfg.filter_config.Min_VrelOncome = 
        conti_ars408_filter_state_cfg_filter_state_min_vrel_oncome_decode(filter_state_cfg_.filter_state_min_vrel_oncome);
    }
    if (conti_ars408_filter_state_cfg_filter_state_min_vrel_depart_is_in_range(filter_state_cfg_.filter_state_min_vrel_depart)){
        can_msg_radar_state_cfg.filter_config.Min_VrelDepart = 
        conti_ars408_filter_state_cfg_filter_state_min_vrel_depart_decode(filter_state_cfg_.filter_state_min_vrel_depart);
    }
    if (conti_ars408_filter_state_cfg_filter_state_min_rcs_is_in_range(filter_state_cfg_.filter_state_min_rcs)){
        can_msg_radar_state_cfg.filter_config.Min_RCS = 
        conti_ars408_filter_state_cfg_filter_state_min_rcs_decode(filter_state_cfg_.filter_state_min_rcs);
    }
    if (conti_ars408_filter_state_cfg_filter_state_min_lifetime_is_in_range(filter_state_cfg_.filter_state_min_lifetime)){
        can_msg_radar_state_cfg.filter_config.Min_Lifetime = 
        conti_ars408_filter_state_cfg_filter_state_min_lifetime_decode(filter_state_cfg_.filter_state_min_lifetime);
    }
    if (conti_ars408_filter_state_cfg_filter_state_min_size_is_in_range(filter_state_cfg_.filter_state_min_size)){
        can_msg_radar_state_cfg.filter_config.Min_Size = 
        conti_ars408_filter_state_cfg_filter_state_min_size_decode(filter_state_cfg_.filter_state_min_size);
    }
    if (conti_ars408_filter_state_cfg_filter_state_min_prob_exists_is_in_range(filter_state_cfg_.filter_state_min_prob_exists)){
        can_msg_radar_state_cfg.filter_config.Min_ProbExists = 
        conti_ars408_filter_state_cfg_filter_state_min_prob_exists_decode(filter_state_cfg_.filter_state_min_prob_exists);
    }
    if (conti_ars408_filter_state_cfg_filter_state_min_y_is_in_range(filter_state_cfg_.filter_state_min_y)){
        can_msg_radar_state_cfg.filter_config.Min_Y = 
        conti_ars408_filter_state_cfg_filter_state_min_y_decode(filter_state_cfg_.filter_state_min_y);
    }
    if (conti_ars408_filter_state_cfg_filter_state_min_x_is_in_range(filter_state_cfg_.filter_state_min_x)){
        can_msg_radar_state_cfg.filter_config.Min_X = 
        conti_ars408_filter_state_cfg_filter_state_min_x_decode(filter_state_cfg_.filter_state_min_x);
    }
    if (conti_ars408_filter_state_cfg_filter_state_min_vy_left_right_is_in_range(filter_state_cfg_.filter_state_max_vy_left_right)){
        can_msg_radar_state_cfg.filter_config.Min_VYRightLeft = 
        conti_ars408_filter_state_cfg_filter_state_min_vy_left_right_decode(filter_state_cfg_.filter_state_min_vy_left_right);
    }
    if (conti_ars408_filter_state_cfg_filter_state_min_vx_oncome_is_in_range(filter_state_cfg_.filter_state_min_vx_oncome)){
        can_msg_radar_state_cfg.filter_config.Min_VXOncome = 
        conti_ars408_filter_state_cfg_filter_state_min_vx_oncome_decode(filter_state_cfg_.filter_state_min_vx_oncome);
    }
    if (conti_ars408_filter_state_cfg_filter_state_min_vy_right_left_is_in_range(filter_state_cfg_.filter_state_min_vy_right_left)){
        can_msg_radar_state_cfg.filter_config.Min_VYRightLeft = 
        conti_ars408_filter_state_cfg_filter_state_min_vy_right_left_decode(filter_state_cfg_.filter_state_min_vy_right_left); 
    }
    if (conti_ars408_filter_state_cfg_filter_state_min_vx_depart_is_in_range(filter_state_cfg_.filter_state_min_vx_depart)){
        can_msg_radar_state_cfg.filter_config.Min_VXDepart = 
        conti_ars408_filter_state_cfg_filter_state_min_vx_depart_decode(filter_state_cfg_.filter_state_min_vx_depart);
    }
    // MAX
    if (conti_ars408_filter_state_cfg_filter_state_max_nof_obj_is_in_range(filter_state_cfg_.filter_state_max_nof_obj)){
        can_msg_radar_state_cfg.filter_config.Max_NofObj = 
        conti_ars408_filter_state_cfg_filter_state_max_nof_obj_decode(filter_state_cfg_.filter_state_max_nof_obj);
    }
    if (conti_ars408_filter_state_cfg_filter_state_max_distance_is_in_range(filter_state_cfg_.filter_state_max_distance)){
        can_msg_radar_state_cfg.filter_config.Max_Distance = 
        conti_ars408_filter_state_cfg_filter_state_max_distance_decode(filter_state_cfg_.filter_state_max_distance);
    }
    if (conti_ars408_filter_state_cfg_filter_state_max_azimuth_is_in_range(filter_state_cfg_.filter_state_max_azimuth)){
        can_msg_radar_state_cfg.filter_config.Max_Azimuth = 
        conti_ars408_filter_state_cfg_filter_state_max_azimuth_decode(filter_state_cfg_.filter_state_max_azimuth);
    }
    if (conti_ars408_filter_state_cfg_filter_state_max_vrel_oncome_is_in_range(filter_state_cfg_.filter_state_max_vrel_oncome)){
        can_msg_radar_state_cfg.filter_config.Max_VrelOncome = 
        conti_ars408_filter_state_cfg_filter_state_max_vrel_oncome_decode(filter_state_cfg_.filter_state_max_vrel_oncome);
    }
    if (conti_ars408_filter_state_cfg_filter_state_max_vrel_depart_is_in_range(filter_state_cfg_.filter_state_max_vrel_depart)){
        can_msg_radar_state_cfg.filter_config.Max_VrelDepart = 
        conti_ars408_filter_state_cfg_filter_state_max_vrel_depart_decode(filter_state_cfg_.filter_state_max_vrel_depart);
    }
    if (conti_ars408_filter_state_cfg_filter_state_max_rcs_is_in_range(filter_state_cfg_.filter_state_max_rcs)){
        can_msg_radar_state_cfg.filter_config.Max_RCS = 
        conti_ars408_filter_state_cfg_filter_state_max_rcs_decode(filter_state_cfg_.filter_state_max_rcs);
    }
    if (conti_ars408_filter_state_cfg_filter_state_max_lifetime_is_in_range(filter_state_cfg_.filter_state_max_lifetime)){
        can_msg_radar_state_cfg.filter_config.Max_Lifetime = 
        conti_ars408_filter_state_cfg_filter_state_max_lifetime_decode(filter_state_cfg_.filter_state_max_lifetime);
    }
    if (conti_ars408_filter_state_cfg_filter_state_max_size_is_in_range(filter_state_cfg_.filter_state_max_size)){
        can_msg_radar_state_cfg.filter_config.Max_Size = 
        conti_ars408_filter_state_cfg_filter_state_max_size_decode(filter_state_cfg_.filter_state_max_size);
    }
    if (conti_ars408_filter_state_cfg_filter_state_max_prob_exists_is_in_range(filter_state_cfg_.filter_state_max_prob_exists)){
        can_msg_radar_state_cfg.filter_config.Max_ProbExists = 
        conti_ars408_filter_state_cfg_filter_state_max_prob_exists_decode(filter_state_cfg_.filter_state_max_prob_exists);
    }
    if (conti_ars408_filter_state_cfg_filter_state_max_y_is_in_range(filter_state_cfg_.filter_state_max_y)){
        can_msg_radar_state_cfg.filter_config.Max_Y = 
        conti_ars408_filter_state_cfg_filter_state_max_y_decode(filter_state_cfg_.filter_state_max_y);
    }
    if (conti_ars408_filter_state_cfg_filter_state_max_x_is_in_range(filter_state_cfg_.filter_state_max_x)){
        can_msg_radar_state_cfg.filter_config.Max_X = 
        conti_ars408_filter_state_cfg_filter_state_max_x_decode(filter_state_cfg_.filter_state_max_x);
    }
    if (conti_ars408_filter_state_cfg_filter_state_max_vy_left_right_is_in_range(filter_state_cfg_.filter_state_max_vy_left_right)){
        can_msg_radar_state_cfg.filter_config.Max_VYRightLeft = 
        conti_ars408_filter_state_cfg_filter_state_max_vy_left_right_decode(filter_state_cfg_.filter_state_max_vy_left_right);
    }
    if (conti_ars408_filter_state_cfg_filter_state_max_vx_oncome_is_in_range(filter_state_cfg_.filter_state_max_vx_oncome)){
        can_msg_radar_state_cfg.filter_config.Max_VXOncome = 
        conti_ars408_filter_state_cfg_filter_state_max_vx_oncome_decode(filter_state_cfg_.filter_state_max_vx_oncome);
    }
    if (conti_ars408_filter_state_cfg_filter_state_max_vy_right_left_is_in_range(filter_state_cfg_.filter_state_max_vy_right_left)){
        can_msg_radar_state_cfg.filter_config.Max_VYRightLeft = 
        conti_ars408_filter_state_cfg_filter_state_max_vy_right_left_decode(filter_state_cfg_.filter_state_max_vy_right_left); 
    }
    if (conti_ars408_filter_state_cfg_filter_state_max_vx_depart_is_in_range(filter_state_cfg_.filter_state_max_vx_depart)){
        can_msg_radar_state_cfg.filter_config.Max_VXDepart = 
        conti_ars408_filter_state_cfg_filter_state_max_vx_depart_decode(filter_state_cfg_.filter_state_max_vx_depart);
    }
}


void ContiController::EncodeMsgCallback_RadarCfg(const radar_driver::Conti_radar_config &msg) {
    CanFrame frame;
    frame.dlc = 8;
    frame.id = 512;
    // std::cout << "Enter the callback of radar_config"<<std::endl;
    struct conti_ars408_radar_configuration_t cmd_RadarCfg;
    cmd_RadarCfg.radar_cfg_max_distance_valid = 
        conti_ars408_radar_configuration_radar_cfg_max_distance_valid_encode(msg.MaxDistance_valid);
    cmd_RadarCfg.radar_cfg_max_distance = 
        conti_ars408_radar_configuration_radar_cfg_max_distance_encode(msg.MaxDistance);
    cmd_RadarCfg.radar_cfg_sensor_id_valid = 
        conti_ars408_radar_configuration_radar_cfg_sensor_id_valid_encode(msg.SensorID_valid);
    cmd_RadarCfg.radar_cfg_sensor_id = 
        conti_ars408_radar_configuration_radar_cfg_sensor_id_encode(msg.SensorID);
    cmd_RadarCfg.radar_cfg_radar_power_valid = 
        conti_ars408_radar_configuration_radar_cfg_radar_power_valid_encode(msg.RadarPower_valid);
    cmd_RadarCfg.radar_cfg_radar_power = 
        conti_ars408_radar_configuration_radar_cfg_radar_power_encode(msg.RadarPower);
    cmd_RadarCfg.radar_cfg_output_type_valid = 
        conti_ars408_radar_configuration_radar_cfg_output_type_valid_encode(msg.OutputType_valid);
    cmd_RadarCfg.radar_cfg_output_type = 
        conti_ars408_radar_configuration_radar_cfg_output_type_encode(msg.OutputType);
    cmd_RadarCfg.radar_cfg_send_quality_valid = 
        conti_ars408_radar_configuration_radar_cfg_send_quality_valid_encode(msg.SendQuality_valid);
    cmd_RadarCfg.radar_cfg_send_quality = 
        conti_ars408_radar_configuration_radar_cfg_send_quality_encode(msg.SendQuality);
    cmd_RadarCfg.radar_cfg_send_ext_info_valid = 
        conti_ars408_radar_configuration_radar_cfg_send_ext_info_valid_encode(msg.SendExtInfo_valid);
    cmd_RadarCfg.radar_cfg_send_ext_info = 
        conti_ars408_radar_configuration_radar_cfg_send_ext_info_encode(msg.SendExtInfo); 
    cmd_RadarCfg.radar_cfg_sort_index_valid = 
        conti_ars408_radar_configuration_radar_cfg_sort_index_valid_encode(msg.SortIndex_valid);
    cmd_RadarCfg.radar_cfg_sort_index = 
        conti_ars408_radar_configuration_radar_cfg_sort_index_encode(msg.SortIndex);
    cmd_RadarCfg.radar_cfg_store_in_nvm_valid = 
        conti_ars408_radar_configuration_radar_cfg_store_in_nvm_valid_encode(msg.StoreInNVM_valid);
    cmd_RadarCfg.radar_cfg_store_in_nvm = 
        conti_ars408_radar_configuration_radar_cfg_store_in_nvm_encode(msg.StoreInNVM);
    cmd_RadarCfg.radar_cfg_ctrl_relay_valid = 
        conti_ars408_radar_configuration_radar_cfg_ctrl_relay_valid_encode(msg.CtrlRelay_valid);
    cmd_RadarCfg.radar_cfg_ctrl_relay = 
        conti_ars408_radar_configuration_radar_cfg_ctrl_relay_encode(msg.CtrlRelay);
    cmd_RadarCfg.radar_cfg_rcs_threshold_valid = 
        conti_ars408_radar_configuration_radar_cfg_rcs_threshold_valid_encode(msg.RCS_Threshold_valid);
    cmd_RadarCfg.radar_cfg_rcs_threshold = 
        conti_ars408_radar_configuration_radar_cfg_rcs_threshold_encode(msg.RCS_Threshold);
    // std::cout << "cmd_RadarCfg has been prepared"<<std::endl;
    
    conti_ars408_radar_configuration_pack(frame.data, &cmd_RadarCfg, sizeof(frame.data));
    SendFunc(&frame);
    // std::cout << "sent radar config can msg"<<std::endl;
    
}

void ContiController::EncodeMsgCallback_FilterCfg(const radar_driver::Conti_filter_config &msg){
    CanFrame frame;
    frame.dlc = 8;
    frame.id = 514;

    struct conti_ars408_filter_cfg_t cmd_FilterCfg;
    cmd_FilterCfg.filter_cfg_type = conti_ars408_filter_cfg_filter_cfg_type_encode(msg.Output_Type);
    cmd_FilterCfg.filter_cfg_valid = conti_ars408_filter_cfg_filter_cfg_valid_encode(msg.Valid);
    cmd_FilterCfg.filter_cfg_active = conti_ars408_filter_cfg_filter_cfg_active_encode(msg.Active);
    cmd_FilterCfg.filter_cfg_index = conti_ars408_filter_cfg_filter_cfg_index_encode(msg.Index);

    cmd_FilterCfg.filter_cfg_min_nof_obj = conti_ars408_filter_cfg_filter_cfg_min_nof_obj_encode(msg.Min_NofObj);
    cmd_FilterCfg.filter_cfg_min_distance = conti_ars408_filter_cfg_filter_cfg_min_distance_encode(msg.Min_Distance);
    cmd_FilterCfg.filter_cfg_min_azimuth = conti_ars408_filter_cfg_filter_cfg_min_azimuth_encode(msg.Min_Azimuth);
    cmd_FilterCfg.filter_cfg_min_vrel_oncome = conti_ars408_filter_cfg_filter_cfg_min_vrel_oncome_encode(msg.Min_VrelOncome);
    cmd_FilterCfg.filter_cfg_min_vrel_depart = conti_ars408_filter_cfg_filter_cfg_min_vrel_depart_encode(msg.Min_VrelDepart);
    cmd_FilterCfg.filter_cfg_min_rcs = conti_ars408_filter_cfg_filter_cfg_min_rcs_encode(msg.Min_RCS);
    cmd_FilterCfg.filter_cfg_min_lifetime = conti_ars408_filter_cfg_filter_cfg_min_lifetime_encode(msg.Min_Lifetime);
    cmd_FilterCfg.filter_cfg_min_size = conti_ars408_filter_cfg_filter_cfg_min_size_encode(msg.Min_Size);
    cmd_FilterCfg.filter_cfg_min_prob_exists = conti_ars408_filter_cfg_filter_cfg_min_prob_exists_encode(msg.Min_ProbExists);
    cmd_FilterCfg.filter_cfg_min_y = conti_ars408_filter_cfg_filter_cfg_min_y_encode(msg.Min_Y);
    cmd_FilterCfg.filter_cfg_min_x = conti_ars408_filter_cfg_filter_cfg_min_x_encode(msg.Min_X);
    cmd_FilterCfg.filter_cfg_min_vy_left_right = conti_ars408_filter_cfg_filter_cfg_min_vy_left_right_encode(msg.Min_VYLeftRight);
    cmd_FilterCfg.filter_cfg_min_vy_right_left = conti_ars408_filter_cfg_filter_cfg_min_vy_right_left_encode(msg.Min_VYRightLeft);
    cmd_FilterCfg.filter_cfg_min_vx_oncome = conti_ars408_filter_cfg_filter_cfg_min_vx_oncome_encode(msg.Min_VXOncome);
    cmd_FilterCfg.filter_cfg_min_vx_depart = conti_ars408_filter_cfg_filter_cfg_min_vx_depart_encode(msg.Min_VXDepart);

    cmd_FilterCfg.filter_cfg_max_nof_obj = conti_ars408_filter_cfg_filter_cfg_max_nof_obj_encode(msg.Max_NofObj);
    cmd_FilterCfg.filter_cfg_max_distance = conti_ars408_filter_cfg_filter_cfg_max_distance_encode(msg.Max_Distance);
    cmd_FilterCfg.filter_cfg_max_azimuth = conti_ars408_filter_cfg_filter_cfg_max_azimuth_encode(msg.Max_Azimuth);
    cmd_FilterCfg.filter_cfg_max_vrel_oncome = conti_ars408_filter_cfg_filter_cfg_max_vrel_oncome_encode(msg.Max_VrelOncome);
    cmd_FilterCfg.filter_cfg_max_vrel_depart = conti_ars408_filter_cfg_filter_cfg_max_vrel_depart_encode(msg.Max_VrelDepart);
    cmd_FilterCfg.filter_cfg_max_rcs = conti_ars408_filter_cfg_filter_cfg_max_rcs_encode(msg.Max_RCS);
    cmd_FilterCfg.filter_cfg_max_lifetime = conti_ars408_filter_cfg_filter_cfg_max_lifetime_encode(msg.Max_Lifetime);
    cmd_FilterCfg.filter_cfg_max_size = conti_ars408_filter_cfg_filter_cfg_max_size_encode(msg.Max_Size);
    cmd_FilterCfg.filter_cfg_max_prob_exists = conti_ars408_filter_cfg_filter_cfg_max_prob_exists_encode(msg.Max_ProbExists);
    cmd_FilterCfg.filter_cfg_max_y = conti_ars408_filter_cfg_filter_cfg_max_y_encode(msg.Max_Y);
    cmd_FilterCfg.filter_cfg_max_x = conti_ars408_filter_cfg_filter_cfg_max_x_encode(msg.Max_X);
    cmd_FilterCfg.filter_cfg_max_vy_left_right = conti_ars408_filter_cfg_filter_cfg_max_vy_left_right_encode(msg.Max_VYLeftRight);
    cmd_FilterCfg.filter_cfg_max_vy_right_left = conti_ars408_filter_cfg_filter_cfg_max_vy_right_left_encode(msg.Max_VYRightLeft);
    cmd_FilterCfg.filter_cfg_max_vx_oncome = conti_ars408_filter_cfg_filter_cfg_max_vx_oncome_encode(msg.Max_VXOncome);
    cmd_FilterCfg.filter_cfg_max_vx_depart = conti_ars408_filter_cfg_filter_cfg_max_vx_depart_encode(msg.Max_VXDepart);

    conti_ars408_filter_cfg_pack(frame.data, &cmd_FilterCfg, sizeof(frame.data));
    SendFunc(&frame);
}

void ContiController::EncodeMsgCallback_Motion(const candata_msgs_pb::CANData &msg) {
    double dir_temp;
    static double threshold = 0.1;    
    ContiController::frame_speed.dlc = 8;
    ContiController::frame_speed.id = 768;
    struct conti_ars408_speed_information_t cmd_speed_;
    
    if (msg.velocity() > threshold){
        dir_temp = 1;
    }else if (msg.velocity() < threshold){
        dir_temp = 2;
    }else {
        dir_temp = 0;
    }
    cmd_speed_.radar_device_speed_direction = conti_ars408_speed_information_radar_device_speed_direction_encode(dir_temp);
    cmd_speed_.radar_device_speed = conti_ars408_speed_information_radar_device_speed_encode(msg.velocity());
    conti_ars408_speed_information_pack(frame_speed.data, &cmd_speed_, sizeof(frame_speed.data));
    speed_update_ = true;
    
    if (speed_update_ && yawrate_update_){
        speed_update_ = false;
        yawrate_update_ = false;
        SendFunc(&frame_speed);
    }
}

void ContiController::EncodeMsgCallback_Yawrate(const polyx_nodea::CorrectedIMU &imu_data){
    ContiController::frame_speed.dlc = 8;
    ContiController::frame_speed.id = 769;
    struct conti_ars408_yaw_rate_information_t cmd_yawrate_;

    cmd_yawrate_.radar_device_yaw_rate = conti_ars408_yaw_rate_information_radar_device_yaw_rate_encode(imu_data.RotationRate.back());
    conti_ars408_speed_information_pack(frame_speed.data, &cmd_speed_, sizeof(frame_speed.data));
    yawrate_update_ = true;
    
    if (speed_update_ && yawrate_update_){
        speed_update_ = false;
        yawrate_update_ = false;
        SendFunc(&frame_speed);
    }
}


} // Namespace radar_driver