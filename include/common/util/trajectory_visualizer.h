#pragma once

#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/MarkerArray.h>

#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoagric {
namespace common {
namespace util {

class TrajectoryVisualizer {
 public:
  TrajectoryVisualizer(ros::NodeHandle& nh);

  virtual ~TrajectoryVisualizer() = default;

  bool Publish(
      const std::unordered_map<std::string, visualization_msgs::MarkerArray>&
          markers);

  visualization_msgs::MarkerArray Arrows(const std::vector<double>& x,
                                         const std::vector<double>& y,
                                         const std::vector<double>& theta);

  visualization_msgs::MarkerArray PointsAndLines(
      const std::vector<double>& x, const std::vector<double>& y,
      const std::vector<double>& theta);

  visualization_msgs::MarkerArray LineStrip(const std::vector<double>& x,
                                            const std::vector<double>& y,
                                            const std::vector<double>& theta);

  visualization_msgs::MarkerArray TextView(
      const std::vector<double>& x, const std::vector<double>& y,
      const std::vector<double>& theta, const std::vector<std::string>& text);

  bool Setup(
      const std::string& name, const std::string frame_id,
      const std::unordered_map<
          std::string, std::pair<std_msgs::ColorRGBA, geometry_msgs::Vector3>>&
          properties,
      const bool has_points_and_lines_ = true, const bool has_arrows = true,
      const bool has_text = false);

 private:
  ros::NodeHandle nh_;

  std::unordered_map<std::string, ros::Publisher> publisher_queue_;

  std::string name_;

  std::string frame_id_;

  std::unordered_map<std::string,
                     std::pair<std_msgs::ColorRGBA, geometry_msgs::Vector3>>
      properties_;

  bool has_properties_ = false;
};

}  // namespace util
}  // namespace common
}  // namespace autoagric
