#include "common/util/trajectory_visualizer.h"

#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>

#include "absl/strings/str_cat.h"
#include "common/macro.h"
#include "common/math/quaternion.h"

namespace autoagric {
namespace common {
namespace util {

using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;

namespace {
geometry_msgs::Pose ToGeometryPose(const double x, const double y,
                                   const double theta) {
  geometry_msgs::Pose pose;

  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = 0.0;
  auto q = common::math::HeadingToQuaternion(theta);
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();

  return pose;
}
}  // namespace

TrajectoryVisualizer::TrajectoryVisualizer(ros::NodeHandle& nh) : nh_(nh) {
  AINFO("Register visualizer " << nh.getNamespace());
}

MarkerArray TrajectoryVisualizer::Arrows(const std::vector<double>& x,
                                         const std::vector<double>& y,
                                         const std::vector<double>& theta) {
  // sanity check
  if (x.size() != y.size() || x.size() != theta.size()) {
    AERROR("States sizes not equal");
    return MarkerArray();
  }

  MarkerArray arrows;
  size_t horizon = x.size();
  arrows.markers.resize(horizon);
  for (size_t i = 0; i < horizon; i++) {
    Marker& arrow = arrows.markers[i];
    arrow.type = Marker::ARROW;
    arrow.pose = ToGeometryPose(x[i], y[i], theta[i]);
    arrow.header.frame_id = frame_id_;
    arrow.ns = "Arrows";
    arrow.id = i;
    arrow.color = properties_["arrows"].first;
    arrow.scale = properties_["arrows"].second;
  }

  return arrows;
}

MarkerArray TrajectoryVisualizer::PointsAndLines(
    const std::vector<double>& x, const std::vector<double>& y,
    const std::vector<double>& theta) {
  // sanity check
  if (x.size() != y.size() || x.size() != theta.size()) {
    AERROR("States sizes not equal");
    return MarkerArray();
  }

  Marker points;
  Marker line_strip;

  size_t horizon = x.size();
  points.header.frame_id = line_strip.header.frame_id = frame_id_;

  points.ns = "Points";
  line_strip.ns = "Line Strip";

  points.action = line_strip.action = Marker::ADD;

  points.id = 0;
  line_strip.id = 1;

  points.type = Marker::POINTS;
  line_strip.type = Marker::LINE_STRIP;

  points.color = line_strip.color = properties_["points_and_lines"].first;
  points.scale = line_strip.scale = properties_["points_and_lines"].second;

  points.points.resize(horizon);
  line_strip.points.resize(horizon);
  for (size_t i = 0; i < horizon; i++) {
    auto pose = ToGeometryPose(x[i], y[i], theta[i]);
    points.points[i] = pose.position;
    line_strip.points[i] = pose.position;
  }

  MarkerArray points_and_lines;
  points_and_lines.markers.emplace_back(points);
  points_and_lines.markers.emplace_back(line_strip);
  return points_and_lines;
}

MarkerArray TrajectoryVisualizer::TextView(
    const std::vector<double>& x, const std::vector<double>& y,
    const std::vector<double>& theta,
    const std::vector<std::string>& text_view) {
  // sanity check
  if (x.size() != y.size() || x.size() != theta.size() ||
      x.size() != text_view.size()) {
    AERROR("States sizes not equal");
    return MarkerArray();
  }

  MarkerArray texts;
  size_t horizon = x.size();
  texts.markers.resize(horizon);
  for (size_t i = 0; i < horizon; i++) {
    Marker& text = texts.markers[i];
    text.type = Marker::TEXT_VIEW_FACING;
    text.pose = ToGeometryPose(x[i], y[i], theta[i]);
    text.header.frame_id = frame_id_;
    text.ns = "Texts";
    text.id = i;
    text.text = text_view[i];
    text.color = properties_["texts"].first;
    text.scale = properties_["texts"].second;
  }

  return texts;
}

bool TrajectoryVisualizer::Setup(
    const std::string& name, const std::string frame_id,
    const std::unordered_map<
        std::string, std::pair<std_msgs::ColorRGBA, geometry_msgs::Vector3>>&
        properties,
    const bool has_points_and_lines_, const bool has_arrows,
    const bool has_text) {
  nh_ = ros::NodeHandle(nh_, name);

  if (has_points_and_lines_) {
    if (properties.find("points_and_lines") == properties.end()) {
      AERROR("No properties preset for points and lines");
      return false;
    }
    publisher_queue_["points_and_lines"] =
        nh_.advertise<MarkerArray>("points_and_lines", 10);
  }

  if (has_arrows) {
    if (properties.find("arrows") == properties.end()) {
      AERROR("No properties preset for arrows");
      return false;
    }
    publisher_queue_["arrows"] = nh_.advertise<MarkerArray>("arrows", 10);
  }

  if (has_text) {
    if (properties.find("texts") == properties.end()) {
      AERROR("No properties preset for texts");
      return false;
    }
    publisher_queue_["texts"] = nh_.advertise<MarkerArray>("texts", 10);
  }

  properties_.clear();
  properties_ = std::move(properties);

  frame_id_.clear();
  frame_id_ = std::move(frame_id);

  has_properties_ = true;
  return true;
}

bool TrajectoryVisualizer::Publish(
    const std::unordered_map<std::string, MarkerArray>& markers) {
  if (!has_properties_) {
    AERROR("No properties yet");
    return false;
  }
  if (markers.size() != publisher_queue_.size()) {
    AERROR("Numbers of markers and publishers not equal");
    return false;
  }
  for (auto& marker_pair : markers) {
    if (publisher_queue_.find(marker_pair.first) == publisher_queue_.end()) {
      AERROR("Cannot find publisher for " << marker_pair.first);
      return false;
    }
    publisher_queue_[marker_pair.first].publish(marker_pair.second);
  }
  return true;
}

}  // namespace util
}  // namespace common
}  // namespace autoagric