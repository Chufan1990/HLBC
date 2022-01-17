#include "common/util/trajectory_visualizer.h"

#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "hlbc");
  ros::NodeHandle nh("~");
  autoagric::common::util::TrajectoryVisualizer trajectory_visualizer(nh);

  ros::spin();

  return 0;
}