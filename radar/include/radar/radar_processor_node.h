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
#include <delphi/EsrTrackArray.h>
#include <delphi/EsrTrack.h>
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


class RadarCompensater{
private:
    std::shared_ptr<ros::NodeHandle> n_;
    ros::Subscriber CAN_sub_;
    ros::Subscriber ESR_sub_;
    ros::Publisher radar_pub_;
    std::unique_ptr<EsrCluster> cluster_;
    std::unique_ptr<tf2_ros::MessageFilter<delphi::EsrTrackArray>> tf2_filter_;

    const std::string map_frame_;
    const std::string ESR_frame_;
    const std::string base_link_;
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_;

    geometry_msgs::Vector3Stamped sensor_velocity_in_ESR;

    bool is_sim_;
    float x_offset = 0.0;
    float y_offset = 0.0;

    custom_msgs::objectList radar_list_;

    void vcu_callback(const custom_msgs::CANData::ConstPtr &msg);
    custom_msgs::objectList
        convert_to_map_frame(const std::shared_ptr<delphi::EsrTrackArray> detections);
    void radar_callback(const delphi::EsrTrackArray::ConstPtr detections);
    void received_radar(const delphi::EsrTrackArray::ConstPtr detections);

    void compensate_ego_motion(const std::shared_ptr<delphi::EsrTrackArray> detections);

public:
    EgoCompensater() = delete;
    EgoCompensater(const EgoCompensater &) = delete;
    EgoCompensater(std::shared_ptr<ros::NodeHandle> nh, bool is_sim, std::string esr_topic_name,
            std::string radar_points_topic_name);
}

#endif //RADAR_PROCESSOR_NODE



