#include "radar/radar_processor_node.h"

// Base_link is the car_frame 
// For vehicle frame : x in along the direction of the vehicle drive 

RadarCompensater::RadarCompensater(
        std::shared_ptr<ros::NodeHandle> nh, bool is_sim = false,
        std::string radadr_topic_name = "/driver/radar/conti/radar_target",
        std::string radar_points_topic_name = "/perception/radar")
: n_(nh), is_sim_(is_sim), buffer_(ros::Duration(100)), listener_(buffer_),
    map_frame_("map"), Radar_frame_("radar"), base_link_("base_link") {

        // Publishers
        // Ouput of radar object in the type of object list (10 is the buffer size)
        radar_pub_ =
            nh->advertise<custom_msgs::objectList>(radar_points_topic_name, 10);

        // Subscribers
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
        // The constructor of the tf2_filter_ 
        // map_frame is the target frame 
        tf2_filter_ = std::unique_ptr< tf2_ros::MessageFilter<radar_driver::Radar_Target> >
            (new tf2_ros::MessageFilter<radar_driver::Radar_Target>(buffer_, map_frame_, 20, *n_));
        tf2_filter_->registerCallback(boost::bind(&RadarCompensater::radar_callback,this,_1));
        
        // get static tramsform from base_link to ESR
        geometry_msgs::TransformStamped radar2baselink;

        while (!buffer_.canTransform(Radar_frame_, base_link_, ros::Time(0))) {
        }
        try {
            radar2baselink=
                buffer_.lookupTransform(base_link_, Radar_frame_, ros::Time(0));
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
        }

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

void RadarCompensater::compensate_ego_motion(const std::shared_ptr<radar_driver::Radar_Target> detections){
    for (auto &radobj: detections->objs_general) {
        float sensor_range_rate = cos(radobj.track_angle - PI/2) * sensor_velocity_in_Radar.vector.x + sin(PI/2 - radobj.track_angle) * sensor_velocity_in_Radar.vector.y;
        radobj.track_range_rate += sensor_range_rate;
        radobj.moving =  fabs(radobj.track_range_rate) < 0.5 ? true:false;
    }
}

custom_msgs::objectList RadarCompensater::convert_to_map_frame(
        const std::shared_ptr<radar_driver::Radar_Target> detected_points) {

    
    ROS_DEBUG("get into convert to map frame, got %lu detections after clustering",detected_points->objs_general.size());
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

    for (auto &radobj : detected_points->objs_general) {
        if(radobj.track_angle > 2.26 || radobj.track_angle < 0.87)
            continue;
        custom_msgs::object converted_point;

        converted_point.tracking_id = radobj.track_ID;
        converted_point.type = 9; // New type for radar points

        /* Centerpoint */
        // Point in ESR frame
        geometry_msgs::PointStamped radar_point, transformed_point;
        radar_point.header = converted_points.header;
        radar_point.point.x =
            radobj.track_range * sin(radobj.track_angle); // x = r*sin(theta)
        radar_point.point.y =
            radobj.track_range * cos(radobj.track_angle); // y = r*cos(theta)
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
        float& range_rate = radobj.track_range_rate;
        //radobj.moving =  fabs(range_rate) < 0.5 ? true:false;
        // transform velocity of radar object to map frame
        geometry_msgs::Vector3Stamped radar_object_velocity,
            transformed_radar_object_velocity;
        // 1. transform magnitude
        radar_object_velocity.header = converted_points.header;
        radar_object_velocity.vector.x = range_rate * sin(radobj.track_angle);
        radar_object_velocity.vector.y = range_rate * cos(radobj.track_angle);
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
        const radar_driver::Radar_Target::ConstPtr detections) {
    ts_ = detections->header.stamp;
    std::shared_ptr<radar_driver::Radar_Target> dets = std::make_shared<radar_driver::Radar_Target>();
    dets->header = detections->header;
    dets->objs_general = detections->objs_general;
    compensate_ego_motion(dets);
    ROS_DEBUG("done ego motion compensation , got %lu detections before clustering",dets->objs_general.size());
    // custom_msgs::objectList radar_points = convert_to_map_frame(std::make_shared<radar_driver::Radar_Target>(cluster_->radar_callback(dets)));
    custom_msgs::objectList radar_points = convert_to_map_frame(std::make_shared<radar_driver::Radar_Target>(dets));
    radar_pub_.publish(radar_points);
}

void RadarCompensater::received_radar(const radar_driver::Radar_Target::ConstPtr detections) {
    tf2_filter_->add(detections);
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
