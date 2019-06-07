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

#define PI 3.14159265

class EsrCluster{
private:
    static const int ESR_RANGE;
    static const int R_RESOLUTION;
    static const float ESR_FOV;
    static const float THETA_RESOLUTION;
    //ros::Subscriber Esr_sub_;
    //ros::Publisher Esr_clu_pub_;
    std::shared_ptr<ros::Publisher> Esr_viz_pub_;
    std::vector<std::vector< int >> W_;
    int f_;
    int h_;
    void initLUT();

    void viz_clustered_marker(const std::vector< std::shared_ptr<delphi::EsrTrack> > clustered, visualization_msgs::MarkerArray &viz_markers_array);
    std::vector< std::shared_ptr<delphi::EsrTrack> > clustering(const std::vector<std::vector<std::shared_ptr< delphi::EsrTrack> > > & detections_grid, std::vector< std::vector<bool> >& visited, int m , int n);// m for row, n for col
    delphi::EsrTrack merge_cluster(const std::vector< std::shared_ptr<delphi::EsrTrack> > &clustered);
    bool InBound(int i,int j ,int r,int c);

public:
    EsrCluster(const EsrCluster &) = delete;
    EsrCluster();
    EsrCluster(int f, int h);
    EsrCluster(const std::shared_ptr<ros::Publisher> esr_viz, int f, int h);//, std::string esr_topic_name, std::string esr_clustered_topic_name);
    radar::Radar_Target radar_callback(const std::shared_ptr<radar::Radar_Target> detections);
};

class RadarCompensater{
private:
    std::shared_ptr<ros::NodeHandle> n_;
    ros::Subscriber CAN_sub_;
    ros::Subscriber ESR_sub_;
    ros::Publisher radar_pub_;
    std::unique_ptr<EsrCluster> cluster_;
    std::unique_ptr<tf2_ros::MessageFilter<radar::Radar_Target>> tf2_filter_;

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
        convert_to_map_frame(const std::shared_ptr<radar::Radar_Target> detections);
    void radar_callback(const radar::Radar_Target::ConstPtr detections);
    void received_radar(const radar::Radar_Target::ConstPtr detections);

    void compensate_ego_motion(const std::shared_ptr<radar::Radar_Target> detections);

public:
    EgoCompensater() = delete;
    EgoCompensater(const EgoCompensater &) = delete;
    EgoCompensater(std::shared_ptr<ros::NodeHandle> nh, bool is_sim, std::string esr_topic_name,
            std::string radar_points_topic_name);
}

#endif //RADAR_PROCESSOR_NODE



