#ifndef RADAR_FACTORY_H
#define RADAR_FACTORY_H

#include<iostream>
#include <ros/ros.h>
#include "radar/conti_ars408/conti_ars408_controller.h"

/**
 * @namespace radar_driver
 * @brief radar_driver
 */
namespace radar_driver {

class RadarControllerFactory {
public:
    static RadarController *create_controller(ros::NodeHandle nh,ros::NodeHandle private_nh);
};

RadarController* RadarControllerFactory::create_controller(ros::NodeHandle nh, ros::NodeHandle private_nh){
    CANCardParameter param;
    bool exit = false;
    if (private_nh.getParam("can_hardware_id", param.hardware_id))
    {
        ROS_INFO("Kvaser CAN Interface - Got hardware_id: %d", param.hardware_id);
        if (param.hardware_id <= 0)
        {
            ROS_ERROR("Kvaser CAN Interface - CAN hardware ID is invalid.");
        exit = true;
        }
    }
    if (private_nh.getParam("can_circuit_id", param.circuit_id))
    {
        ROS_INFO("Kvaser CAN Interface - Got can_circuit_id: %d", param.circuit_id);

        if (param.circuit_id < 0)
        {
        ROS_ERROR("Kvaser CAN Interface - Circuit ID is invalid.");
        exit = true;
        }
    }
    if (private_nh.getParam("can_bit_rate", param.bit_rate))
    {
        ROS_INFO("Kvaser CAN Interface - Got bit_rate: %d", param.bit_rate);
        
        if (param.bit_rate < 0)
        {
        ROS_ERROR("Kvaser CAN Interface - Bit Rate is invalid.");
        exit = true;
        }
    }
    private_nh.param("radar_model", param.radar_model, std::string("conti_ars408"));
    private_nh.param("radar_mode", param.radar_mode, std::string("object"));
    if (exit)
        return nullptr;
    if (param.radar_model == "conti_ars408")
    {
        return new ContiController(param);
    }
}

} // namespace radar_driver

#endif