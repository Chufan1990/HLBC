#include <ros/ros.h>

#include "control/common/trajectory_visualizer.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "hlbc");
  ros::NodeHandle nh("~");
  autoagric::control::common::TrajectoryVisualizer trajectory_visualizer(nh);

  ros::spin();

  return 0;
}