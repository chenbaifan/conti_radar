#include "radar_visualizer.h"

namespace radar{

RadarVisualizer::RadarVisualizer(/* args */)
{
    ;
}

RadarVisualizer::~RadarVisualizer()
{
    ;
}

visualization_msgs::MarkerArray RadarVisualizer::Update(std::shared_ptr<radar_driver::RadarTrackArray> trackarray_, 
std::string frame_id_)
{
    int i;
    visualization_msgs::MarkerArray markerarray_;
    visualization_msgs::Marker deleteall_marker;
    deleteall_marker.action = visualization_msgs::Marker::DELETEALL;    
    for (i = 0; i<trackarray_->tracks.size() ; i++){
        visualization_msgs::Marker marker_;
        marker_.ns = "radar_obj";
        marker_.header.frame_id = frame_id_;
        marker_.header.stamp = trackarray_->header.stamp;
        marker_.id = trackarray_->tracks[i].track_ID;
        marker_.pose.position.x = trackarray_->tracks[i].track_dist_long;
        marker_.pose.position.y = trackarray_->tracks[i].track_dist_lat;
        marker_.pose.position.z = 1;
        marker_.pose.orientation.w = 1;
        marker_.action = visualization_msgs::Marker::ADD;
        marker_.type = visualization_msgs::Marker::CUBE;
        marker_.scale.x = 1;
        marker_.scale.y = 1;
        marker_.scale.z = 2;
        marker_.color.r = 1.0;
        marker_.color.g = 0.5;
        marker_.color.b = 0.8;
        marker_.color.a = 1;
        marker_.text = std::to_string(trackarray_->tracks[i].track_vrel_long);
        markerarray_.markers.push_back(marker_);
    }
    
}

}//namespace radar