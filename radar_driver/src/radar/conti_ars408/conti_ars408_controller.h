#ifndef CONTI_ARS408_CONTROLLER_H
#define CONTI_ARS408_CONTROLLER_H

#include "radar/radar_controller.h"
#include "conti_ars408.h"
#include "std_msgs/UInt8.h"
#include <radar_driver/Radar_Target.h>
#include <radar_driver/Radar_State_Cfg.h>
#include <radar_driver/RadarTrackArray.h>
#include <pb_msgs/candata_pb.pb.h>
#include <polyx_nodea/CorrectedIMU.h>
#include <string>
#include <algorithm>
#include <ros/ros.h>

/**
 * @namespace radar_driver
 * @brief radar_driver
 */
namespace radar_driver {

/**
 * @class ContiController
 * @brief Derived class which inits radar can parameters, 
 * Decodes and encodes can ros messages  
 */
class ContiController:public RadarController{
public:
    /**
    * @brief Constructor
    */
    explicit ContiController(const CANCardParameter &param);
    /**
    * @brief Destructor
    */
    ~ContiController() = default;
    /**
    * @brief init can_reader, can_writer, RecvThreadFunc, publisher & subscribers 
    */
    void init(ros::NodeHandle &nh) override;
    /**
    * @brief Overide this function in derived Class
    * called from can_recv_thread_ to read can frames from can bus
    */
    void RecvThreadFunc() override;

private:
    /**
    * @brief Called to decode radar can frame data to ros obj/cluster msg  
    */
    void DecodeMsgPublish(const CanFrame& frame);

    void DecodeObjectStatus(const CanFrame &frame);
    void DecodeObjectGeneral(const CanFrame &frame);
    void DecodeObjectQuality(const CanFrame &frame);
    void DecodeObjectExtended(const CanFrame &frame);
    
    void DecodeClusterStatus(const CanFrame &frame);
    void DecodeClusterGeneral(const CanFrame &frame);
    void DecodeClusterQuality(const CanFrame &frame);
    
    void DecodeRadarState(const CanFrame &frame);
    void DecodeFilterCfgHeader(const CanFrame &frame);
    void DecodeFilterCfg(const CanFrame &frame);
    void DecodeSoftVersionId(const CanFrame &frame);
    
    // 
    void Combine2TrackArray();
    // Encode msg 
    void EncodeMsgCallback_RadarCfg(const radar_driver::Conti_radar_config &msg);
    void EncodeMsgCallback_FilterCfg(const radar_driver::Conti_filter_config &msg);
    void EncodeMsgCallback_Motion(const candata_msgs_pb::CANData &msg);
    void EncodeMsgCallback_Yawrate(const polyx_nodea::CorrectedIMU &imu_data);

    //used for encoding in frame id 16
    //int motion_control_counter_ = 0;

    // Output Mode
    std::string output_mode;

    //Ros can msg
    radar_driver::Radar_Target radar_target_;
    radar_driver::RadarTrackArray can_msg_radar_track_array;
    radar_driver::Radar_State_Cfg can_msg_radar_state_cfg;
    bool radar_target_start_update_;
    bool radar_target_quality_update_;
    bool radar_target_end_update_;
    bool radar_state_update_;
    bool radar_quality_enable_;
    bool radar_extended_enable_;
    int radar_output_mode_;
    int num_of_target_;
    bool speed_update_;
    bool yawrate_update_;

    conti_ars408_obj_0_status_t obj_0_status_; 
    conti_ars408_obj_1_general_t obj_1_general_;
    conti_ars408_obj_2_quality_t obj_2_quality_;
    conti_ars408_obj_3_extended_t obj_3_extended_;  

    conti_ars408_cluster_0_status_t cluster_0_status_;
    conti_ars408_cluster_1_general_t cluster_1_general_;
    conti_ars408_cluster_2_quality_t cluster_2_quality_;
    
    conti_ars408_radar_state_t radar_state_;
    conti_ars408_filter_state_header_t filter_state_header_;
    conti_ars408_filter_state_cfg_t filter_state_cfg_;
    

    // Define can_frame for speed and yaw_rate send at the same time
    CanFrame frame_speed, frame_yawrate;
    struct conti_ars408_speed_information_t cmd_speed_;
    struct conti_ars408_yaw_rate_information_t cmd_yawrate_;
    //Ros can msg
    // Can decode&encode msg define 
    //DISALLOW_COPY_AND_ASSIGN(ContiController);
};

} // namespace radar_driver

#endif //