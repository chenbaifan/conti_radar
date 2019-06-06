#include "ros/ros.h"
#include <radar/radar_factory.h>

int main(int argc, char** argv) {

  ROS_INFO("Conti ARS408");
  ros::init(argc, argv, "radar_conti_node");
  ros::NodeHandle nh("radar");
  ros::NodeHandle private_nh("~");
  ros::AsyncSpinner spinner(1);

// Wait for time to be valid
  while (ros::Time::now().nsec == 0);
  // start the radar_controller
  radar_driver::RadarController* radar_controller = 
  radar_driver::RadarControllerFactory::create_controller(nh,private_nh);
  if (radar_controller == nullptr){
    ROS_BREAK();
  }
  radar_controller->init(nh);
  ros::spin();
  return 0;
}

