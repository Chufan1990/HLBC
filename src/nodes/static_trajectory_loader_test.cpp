#include "common/test/static_trajectory_loader.h"

#include <stdlib.h>

#include <string>

#include "absl/strings/str_cat.h"
#include "common/macro.h"
#include "common/util/file.h"
#include "planning/common/planning_gflags.h"
#include "ros/ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "trajectory_loader");

  ros::NodeHandle nh("~");

  std::string path = absl::StrCat(std::string(std::getenv("HOME")),
                                  "/autoware.ai/src/autoware/common/"
                                  "hlbc/data/saved_waypoints.csv");
  autoagric::common::test::StaticTrajectoryLoader loader(nh, path);

  loader.Init();

  autoagric::planning::TrajectorySmootherConfig trajectory_smoother_conf;

  if (!autoagric::common::util::GetProtoFromFile(
          FLAGS_discrete_points_smoother_config_filename,
          &trajectory_smoother_conf)) {
    AERROR("Unable to load control conf file: "
           << FLAGS_discrete_points_smoother_config_filename);
  }

  loader.Smooth(trajectory_smoother_conf);

  geometry_msgs::Vector3 scale;
  std_msgs::ColorRGBA color;

  scale.x = 0.05;
  scale.y = 0.05;
  color.r = 0.0;
  color.g = 1.0;
  color.b = 1.0;
  color.a = 1.0;

  loader.InitVisualizer("global", scale, color);

  ros::spin();

  return 0;
}