#include <stdlib.h>

#include <string>

#include "absl/strings/str_cat.h"
#include "autoagric/planning/static_path_config.pb.h"
#include "common/macro.h"
#include "common/util/file.h"
#include "common/util/static_path_wrapper.h"
#include "planning/common/planning_gflags.h"
#include "ros/ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "trajectory_loader");

  ros::NodeHandle nh("~");

  std::string data;
  std::string config;

  nh.getParam("data", data);
  nh.getParam("config", config);

  std::string data_filename =
      absl::StrCat(std::string(std::getenv("HOME")), data);
  std::string static_path_config_filename =
      absl::StrCat(std::string(std::getenv("HOME")), config);

  autoagric::common::util::StaticPathWrapper static_path_wrapper(nh,
                                                                 data_filename);

  autoagric::planning::StaticPathConfig static_path_conf;

  if (!autoagric::common::util::GetProtoFromFile(static_path_config_filename,
                                                 &static_path_conf)) {
    AERROR("Unable to load control conf file: " << static_path_config_filename);
    return 1;
  }

  if (!static_path_wrapper.Init(static_path_conf)) {
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