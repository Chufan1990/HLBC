#include <stdlib.h>

#include <string>

#include "absl/strings/str_cat.h"
#include "common/macro.h"
#include "common/util/file.h"
#include "common/util/static_path_wrapper.h"
#include "planning/common/planning_gflags.h"
#include "ros/ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "trajectory_loader");

  ros::NodeHandle nh("~");

  std::string filename;

  nh.getParam("filename", filename);

  std::string path = absl::StrCat(std::string(std::getenv("HOME")), filename);

  autoagric::common::util::StaticPathWrapper static_path_wrapper(nh, path);

  autoagric::planning::TrajectorySmootherConfig trajectory_smoother_conf;

  if (!autoagric::common::util::GetProtoFromFile(
          FLAGS_discrete_points_smoother_config_filename,
          &trajectory_smoother_conf)) {
    AERROR("Unable to load control conf file: "
           << FLAGS_discrete_points_smoother_config_filename);
  }

  if (!static_path_wrapper.Init(trajectory_smoother_conf)) {
    AERROR("StaticPathWrapper init failed");
    return 1;
  }

  geometry_msgs::Vector3 scale;
  std_msgs::ColorRGBA color;

  scale.x = 0.05;
  scale.y = 0.05;
  color.r = 0.0;
  color.g = 1.0;
  color.b = 1.0;
  color.a = 1.0;

  static_path_wrapper.InitVisualizer("global", scale, color);

  if (!static_path_wrapper.Proc()) {
    AERROR("StaticPathWrapper process failed");
    return 1;
  }

  return 0;
}