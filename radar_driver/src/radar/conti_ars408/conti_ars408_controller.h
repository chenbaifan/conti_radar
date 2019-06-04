#ifndef CONTI_ARS408_CONTROLLER_H
#define CONTI_ARS408_CONTROLLER_H

#include "radar/radar_controller.h"
#include "conti_ars408.h"
#include <pb_msgs/candata_pb.pb.h>
/**
 * @namespace saic::drivers::canbus
 * @brief saic::drivers::canbus
 */
namespace radar_driver {

/**
 * @class GemController
 * @brief Derived class which inits vehicle can parameters, 
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
    * @brief Called to decode GEM vehicle can frame data to ros controll
    * feedback 
    */
    void DecodeMsgPublish(const CanFrame& frame);
    /**
    * @brief Called to encode ros command msgto  GEM vehicle can frame data 
    */
    void EncodeMsgCallback(const candata_msgs_pb::CANData& msg);

    void EncodeMsgAuxCallback(const candata_msgs_pb::Auxiliary& msg);

    //used for encoding in frame id 16
    int motion_control_counter_ = 0;

    //Ros can msg
    candata_msgs_pb::CANData can_msg_;
    bool fbk_frame32_updated_;
    bool fbk_frame35_updated_;

    // Can decode&encode msg define 
    //can feedback msg1
    motivo_rexus_llc_motion_feedback1_t motion_fbk1_;

    //can feedback msg1
    motivo_rexus_llc_motion_feedback2_t motion_fbk2_;

    conti_ars408_obj_0_status_t obj_status_;


    DISALLOW_COPY_AND_ASSIGN(ContiController);
};

} // namespace radar_driver

#endif //GEM_CONTROLLER_H