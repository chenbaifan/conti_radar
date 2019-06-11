#include "radar/radar_processor_node.h"

// Base_link is the car_frame 
// For vehicle frame : x in along the direction of the vehicle drive 
static ros::Time ts_ = ros::Time();


RadarCompensater::RadarCompensater(
        std::shared_ptr<ros::NodeHandle> nh, bool is_sim = false,
        std::string radar_topic_name = "/driver/radar/conti/radar_target",
        std::string radar_points_topic_name = "/perception/radar", 
        std::string radar_config_topic_name = "/Perception/radar/radar_cfg",
        std::string radar_config_state_topic_name = "/driver/radar/conti/radar_state")
: n_(nh), is_sim_(is_sim), buffer_(ros::Duration(100)), listener_(buffer_),
    map_frame_("map"), Radar_frame_("radar_conti"), base_link_("base_link"){

// For_Debug
        ROS_INFO("Enter the construction function");
        nh->param("radar_mode", radar_mode_, std::string("object"));
        radar_config_.SortIndex = 1;
        radar_config_.MaxDistance = 260;
        radar_config_.SensorID = 0;
        radar_config_.RadarPower = 0;
        radar_config_.RCS_Threshold = 1;
        radar_config_.SendQuality = true;
        radar_config_.SendExtInfo = true;
        if (radar_mode_ == "object"){
            radar_config_.OutputType = 1; // 0: nothing   1:object  2:cluster 
        }else if (radar_mode_ == "cluster"){
            radar_config_.OutputType = 2; // 0: nothing   1:object  2:cluster 
        }
        radar_config_.OutputType = 2;

        // Publishers
        // Ouput of radar object in the type of object list (10 is the buffer size)
        radar_config_pub_ = 
            nh->advertise<radar_driver::Conti_radar_config>(radar_config_topic_name, 1);
        radar_pub_ =
            nh->advertise<custom_msgs::objectList>(radar_points_topic_name, 10);

        // Subscribers
        Radar_config_state_sub_ = 
            nh->subscribe(radar_config_state_topic_name, 10, &RadarCompensater::radar_config_callback, this);
        // Get the vehicle information 
        if (n_->hasParam("/sim")) {
            n_->getParam("/sim", is_sim_);
            if (is_sim_)
                CAN_sub_ = nh->subscribe("/saicic/vcu/simulated_vcu_feedback", 10,
                        &RadarCompensater::vcu_callback, this);
            else
                CAN_sub_ = nh->subscribe("/vehicle/canFeedback", 10,
                        &RadarCompensater::vcu_callback, this);
        } else
            CAN_sub_ = nh->subscribe("/vehicle/canFeedback", 10,
                    &RadarCompensater::vcu_callback, this);

        Radar_sub_ =
            nh->subscribe(radar_topic_name, 10, &RadarCompensater::received_radar, this);
        
// For_Debug
        ROS_INFO("Initialize subscriber");

        // The constructor of the tf2_filter_ 
        // map_frame is the target frame 
        tf2_filter_ = std::unique_ptr< tf2_ros::MessageFilter<radar_driver::RadarTrackArray> >
            (new tf2_ros::MessageFilter<radar_driver::RadarTrackArray>(buffer_, map_frame_, 20, *n_));
        tf2_filter_->registerCallback(boost::bind(&RadarCompensater::radar_callback,this,_1));
        
        // get static tramsform from base_link to ESR
        geometry_msgs::TransformStamped radar2baselink;

// For_Debug
        ROS_INFO("Initializing  tf2");

        while (!buffer_.canTransform(Radar_frame_, base_link_, ros::Time(0))) {
            //ROS_INFO("canTransform");
        }
        try {
            radar2baselink=
                buffer_.lookupTransform(base_link_, Radar_frame_, ros::Time(0));
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
        }

// For_Debug
        ROS_INFO("Initialized tfc");
        
        x_offset = radar2baselink.transform.translation.x;
        y_offset = radar2baselink.transform.translation.y;
        //geometry_msgs::PointStamped origin,transformed;
        //tf2::doTransform(origin, transformed, radar2baselink);
        //x_offset = transformed.point.x;
        //y_offset = transformed.point.y;
//FOR DEBUG
        // std::shared_ptr<ros::Publisher> cluster_debug_viz =nullptr;
//FOR DEBUG
        //cluster_debug_viz = std::make_shared<ros::Publisher> (nh->advertise<visualization_msgs::MarkerArray> (radar_points_topic_name+"/viz", 10));
//FOR DEBUG
        //cluster_ = std::unique_ptr<EsrCluster>(new EsrCluster(cluster_debug_viz));
    }

// Get the speed transformation from radar_frame to vehicle_frame
void RadarCompensater::vcu_callback(const custom_msgs::CANData::ConstPtr &msg) {
    float ego_speed_ = msg->velocity;
    float curvature = msg->curvature;
    float angular_velocity_ = curvature * ego_speed_;

    geometry_msgs::Vector3Stamped sensor_velocity_in_baselink;
    sensor_velocity_in_baselink.header.frame_id = base_link_;
    sensor_velocity_in_baselink.header.stamp = msg->header.stamp;
    sensor_velocity_in_baselink.vector.x =
        ego_speed_ - angular_velocity_ * y_offset;
    sensor_velocity_in_baselink.vector.y =
        angular_velocity_ * x_offset;
    sensor_velocity_in_baselink.vector.z = 0;

    // Rotate the above velocity from  base_link to ESR frame
    try {
        sensor_velocity_in_Radar =
            buffer_.transform(sensor_velocity_in_baselink, Radar_frame_);
    } catch (tf2::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
    }
}

void RadarCompensater::compensate_ego_motion(const std::shared_ptr<radar_driver::RadarTrackArray> detections){
    for (auto &radobj: detections->tracks) {
        radobj.track_vrel_long += sensor_velocity_in_Radar.vector.x;
        radobj.track_vrel_lat += sensor_velocity_in_Radar.vector.y;
        //float sensor_range_rate = cos(radobj.track_angle - PI/2) * sensor_velocity_in_Radar.vector.x + sin(PI/2 - radobj.track_angle) * sensor_velocity_in_Radar.vector.y;
        //radobj.track_range_rate += sensor_range_rate;
        //radobj.moving =  fabs(radobj.track_range_rate) < 0.5 ? true:false;
        
    }
}

custom_msgs::objectList RadarCompensater::convert_to_map_frame(
        const std::shared_ptr<radar_driver::RadarTrackArray> detected_points) {

    
    ROS_DEBUG("get into convert to map frame, got %lu detections after clustering",detected_points->tracks.size());
    custom_msgs::objectList converted_points;

    converted_points.header = detected_points->header;
    converted_points.num_objs = 0;

    // get all transforms here
    geometry_msgs::TransformStamped esr2map;
    try {
        esr2map = buffer_.lookupTransform(map_frame_, Radar_frame_, detected_points->header.stamp);
    } catch (tf2::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
    }

    for (auto &radobj : detected_points->tracks) {
        if(radobj.track_orientation_angle < -1.0472 || radobj.track_orientation_angle > 1.0472)
            continue;
        custom_msgs::object converted_point;

        converted_point.tracking_id = radobj.track_ID;
        converted_point.type = 9; // New type for radar points

        /* Centerpoint */
        // Point in ESR frame
        geometry_msgs::PointStamped radar_point, transformed_point;
        radar_point.header = converted_points.header;
        radar_point.point.x = radobj.track_dist_long; // x = r*sin(theta)
        radar_point.point.y = radobj.track_dist_lat; // y = r*cos(theta)
        radar_point.point.z = 0;

        // Convert to Map frame
        tf2::doTransform(radar_point, transformed_point, esr2map);

        converted_point.centerPoint = {(float)transformed_point.point.x,
            (float)transformed_point.point.y};

        /* Bounding Box - No size */
        converted_point.points = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        /* Velocity */
        // Velocity of radar object in ESR frame

        //float sensor_range_rate = cos(radobj.track_angle - PI/2) * sensor_velocity_in_Radar.vector.x + sin(PI/2 - radobj.track_angle) * sensor_velocity_in_Radar.vector.y;
        //float& range_rate = radobj.track_range_rate;
        //radobj.moving =  fabs(range_rate) < 0.5 ? true:false;
        // transform velocity of radar object to map frame
        geometry_msgs::Vector3Stamped radar_object_velocity,
            transformed_radar_object_velocity;
        // 1. transform magnitude
        radar_object_velocity.header = converted_points.header;
        radar_object_velocity.vector.x = radobj.track_vrel_long;
        radar_object_velocity.vector.y = radobj.track_vrel_lat;
        radar_object_velocity.vector.z = 0;
        //ROS_INFO("object id %d, range_rate is %f, compensated range_rate is %f velocity in ESR frame after compensate:(%f, %f), sensor motion is :(%f, %f)",radobj.track_ID,radobj.track_range_rate, range_rate, radar_object_velocity.vector.x, radar_object_velocity.vector.y, sensor_velocity_in_Radar.vector.x, sensor_velocity_in_Radar.vector.y);
        // 2. Rotate to map frame
        tf2::doTransform(radar_object_velocity, transformed_radar_object_velocity,
                esr2map);

        converted_point.velocity = {(float)transformed_radar_object_velocity.vector.x,
            (float)transformed_radar_object_velocity.vector.y};

        converted_point.velocity_var = {0.5, 0.5};

        converted_points.objs.push_back(converted_point);
    }
    converted_points.num_objs = converted_points.objs.size();

    return converted_points;
}

void RadarCompensater::radar_callback(
        const radar_driver::RadarTrackArray::ConstPtr detections) {
    ts_ = detections->header.stamp;
    std::shared_ptr<radar_driver::RadarTrackArray> dets = std::make_shared<radar_driver::RadarTrackArray>();
    dets->header = detections->header;
    dets->tracks = detections->tracks;
    compensate_ego_motion(dets);
    ROS_DEBUG("done ego motion compensation , got %lu detections before clustering",dets->tracks.size());
    // custom_msgs::objectList radar_points = convert_to_map_frame(std::make_shared<radar_driver::RadarTrackArray>(cluster_->radar_callback(dets)));
    // custom_msgs::objectList radar_points = convert_to_map_frame(std::make_shared<radar_driver::RadarTrackArray>(dets));
    custom_msgs::objectList radar_points = convert_to_map_frame(dets);
    radar_pub_.publish(radar_points);
}

void RadarCompensater::received_radar(const radar_driver::RadarTrackArray::ConstPtr detections) {
    // ROS_INFO("Received data from radar");

    tf2_filter_->add(detections);
}

void RadarCompensater::radar_config_callback(const radar_driver::Radar_State_Cfg radar_state_cfg_){
    if (!RadarCompensater::radar_configed_check(radar_state_cfg_.radar_state)){
        radar_config_.header.stamp = ros::Time::now();
        radar_config_pub_.publish(radar_config_);
    }
};

bool RadarCompensater::radar_configed_check(const radar_driver::Conti_radar_state radar_state_){
    bool temp_flag = true;
    if (radar_config_.SortIndex != radar_state_.SortIndex){
        radar_config_.SortIndex_valid  = true;
        temp_flag = false;
    }
    if (radar_config_.MaxDistance != radar_state_.MaxDistanceCfg){
        radar_config_.MaxDistance_valid = true;
        temp_flag = false;
    }
    if (radar_config_.SensorID != radar_state_.SensorID){
        radar_config_.SensorID_valid = true;
        temp_flag = false;
    }
    if (radar_config_.RadarPower != radar_state_.RadarPowerCfg){
        radar_config_.RadarPower_valid = true;
        temp_flag = false;
    }
    if (radar_config_.RCS_Threshold != radar_state_.RCS_Threshold){
        radar_config_.RCS_Threshold_valid = true;
        temp_flag = false;
    }
    if (radar_config_.SendQuality != radar_state_.SendQualityCfg){
        radar_config_.SendQuality_valid = true;
        temp_flag = false;
    }
    if (radar_config_.SendExtInfo != radar_state_.SendExtInfoCfg){
        radar_config_.SendExtInfo_valid = true;
        temp_flag = false;
    }
    if (radar_config_.OutputType != radar_state_.OutputTypeCfg){
        radar_config_.OutputType_valid = true;
        temp_flag = false;
    }
    return temp_flag;   
}

int main(int argc, char** argv) {
    ROS_INFO("Radar signal process");
    ros::init(argc, argv, "radar_processor_node");
    std::shared_ptr<ros::NodeHandle> n = std::make_shared<ros::NodeHandle>();
//   ros::NodeHandle nh("radar_processor");
//   ros::NodeHandle private_nh("~");
    RadarCompensater compensater(n);
    ros::spin();
    return 0;
}
