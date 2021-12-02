#include "ros/ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "control");
  ros::NodeHandle nh("~");
  autoagric::control::ControlComponent control_component(nh);
  ros::spin();
  return 0;
}