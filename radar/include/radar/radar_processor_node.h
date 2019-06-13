#ifndef RADAR_PROCESSOR_NODE
#define RADAR_PROCESSOR_NODE

#include <ros/ros.h>
#include <time.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <custom_msgs/CANData.h>
#include <radar_driver/Radar_State_Cfg.h>
#include <radar_driver/RadarTrackArray.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <mutex>
#include <string>
#include <math.h>
#include <unordered_set>
#include <custom_msgs/CANData.h>
#include <custom_msgs/object.h>
#include <custom_msgs/objectList.h>
#include <memory>
#include <queue>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <unistd.h>
#include "boost/filesystem.hpp"
#include "radar_visualizer.h"

#define PI 3.14159265
namespace bfs = boost::filesystem;

namespace radar{

class RadarCompensater{
private:
    std::shared_ptr<ros::NodeHandle> n_;
    ros::Subscriber CAN_sub_;
    ros::Subscriber Radar_sub_;
    ros::Subscriber Radar_config_state_sub_;
    ros::Publisher radar_pub_;
    ros::Publisher radar_config_pub_;
    ros::Publisher viz_pub_raw_;
    ros::Publisher viz_pub_processed_;
    //std::unique_ptr<EsrCluster> cluster_;
    std::unique_ptr<tf2_ros::MessageFilter<radar_driver::RadarTrackArray>> tf2_filter_;

    const std::string map_frame_;
    const std::string Radar_frame_;
    const std::string base_link_;
    std::string radar_mode_;
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_;

    geometry_msgs::Vector3Stamped sensor_velocity_in_Radar;

    bool is_sim_;
    bool visual_raw_;
    bool visual_processed_;
    float x_offset = 0.0;
    float y_offset = 0.0;

    // Initialize the yaml parser for radar config
    std::string config_path;

    RadarVisualizer visualizer;
    

    custom_msgs::objectList radar_list_;
    radar_driver::Conti_radar_config radar_config_;

    void vcu_callback(const custom_msgs::CANData::ConstPtr &msg);
    custom_msgs::objectList
        convert_to_map_frame(const std::shared_ptr<radar_driver::RadarTrackArray> detections);
    void radar_callback(const radar_driver::RadarTrackArray::ConstPtr detections);
    void received_radar(const radar_driver::RadarTrackArray::ConstPtr detections);
    void radar_config_callback(const radar_driver::Radar_State_Cfg radar_cfg_state_);
    bool radar_configed_check(const radar_driver::Conti_radar_state radar_state_);
    void compensate_ego_motion(const std::shared_ptr<radar_driver::RadarTrackArray> detections);
    void load_radar_config(const ros::NodeHandle private_nh);
public:
    RadarCompensater() = delete;
    RadarCompensater(const RadarCompensater &) = delete;
    RadarCompensater(std::shared_ptr<ros::NodeHandle> nh, ros::NodeHandle private_nh, bool is_sim, 
            bool visual_raw, bool visual_processed, std::string esr_topic_name, std::string radar_points_topic_name, 
            std::string radar_config_topic_name, std::string radar_config_state_topic_name);
};

} //namespace radar

#endif //RADAR_PROCESSOR_NODEROS_INFO("Rec");