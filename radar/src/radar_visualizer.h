#ifndef RADAR_VISUALIZER
#define RADAR_VISUALIZER

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <radar_driver/RadarTrackArray.h>
#include <ros/ros.h>
#include <iostream>

namespace radar{

class RadarVisualizer
{
private:
    /* data */
public:
    RadarVisualizer(/* args */);
    ~RadarVisualizer();
    visualization_msgs::MarkerArray Update(const radar_driver::RadarTrackArray::ConstPtr trackarray_, std::string frame_id_);
};

} //namespace radar

#endif //RADAR_VISUALIZER