#include "common/util/trajectory_visualizer.h"

#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>

#include "absl/strings/str_cat.h"
#include "common/macro.h"
#include "common/math/box2d.h"
#include "common/math/quaternion.h"

namespace autoagric {
namespace common {
namespace util {

using common::math::Box2d;
using common::math::Vec2d;
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
    const bool has_texts, const bool has_boundingboxs,
    const bool has_obstacles) {
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

  if (has_texts) {
    if (properties.find("texts") == properties.end()) {
      AERROR("No properties preset for texts");
      return false;
    }
    publisher_queue_["texts"] = nh_.advertise<MarkerArray>("texts", 10);
  }

  if (has_boundingboxs) {
    if (properties.find("boundingboxs") == properties.end()) {
      AERROR("No properties preset for bounding boxs");
      return false;
    }
    publisher_queue_["boundingboxs"] =
        nh_.advertise<MarkerArray>("boundingbox", 10);
  }

  if (has_obstacles) {
    if (properties.find("obstacles") == properties.end()) {
      AERROR("No properties preset for obstacles");
      return false;
    }
    publisher_queue_["obstacles"] = nh_.advertise<MarkerArray>("obstacles", 10);
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

MarkerArray TrajectoryVisualizer::BoundingBoxs(
    const std::vector<double>& x, const std::vector<double>& y,
    const std::vector<double>& theta, const common::VehicleParam& vehicle_param,
    const size_t interval) {
  if (x.size() != y.size() || x.size() != theta.size()) {
    AERROR("States sizes not equal");
    return MarkerArray();
  }

  MarkerArray boundingboxs;

  const size_t horizon = x.size();

  for (size_t i = 0; i < horizon; i += interval) {
    auto bbox = EgoBox(x[i], y[i], theta[i], vehicle_param);
    bbox.id = i;
    boundingboxs.markers.emplace_back(std::move(bbox));
  }

  return boundingboxs;
}

Marker TrajectoryVisualizer::EgoBox(const double x, const double y,
                                    const double theta,
                                    const common::VehicleParam& vehicle_param) {
  const double ego_length = vehicle_param.length();
  const double ego_width = vehicle_param.width();

  const double shift_distance =
      ego_length / 2.0 - vehicle_param.back_edge_to_center();

  Box2d ego_box({x + shift_distance * std::cos(theta),
                 y + shift_distance * std::sin(theta)},
                theta, ego_length, ego_width);

  std::vector<Vec2d> ego_box_corners = ego_box.GetAllCorners();

  Marker boundingbox;

  boundingbox.type = Marker::LINE_STRIP;
  boundingbox.header.frame_id = frame_id_;
  boundingbox.ns = "bounding boxs";
  boundingbox.action = Marker::ADD;
  boundingbox.color = properties_["boundingboxs"].first;
  boundingbox.scale = properties_["boundingboxs"].second;
  if (!ego_box_corners.empty()) {
    for (const auto& vertice : ego_box_corners) {
      auto point = ToGeometryPose(vertice.x(), vertice.y(), 0.0).position;
      boundingbox.points.emplace_back(point);
    }
    boundingbox.points.emplace_back(ToGeometryPose(ego_box_corners.front().x(),
                                                   ego_box_corners.front().y(),
                                                   0.0)
                                        .position);
  }
  return boundingbox;
}

MarkerArray TrajectoryVisualizer::BoundingBoxs(
    const std::vector<std::vector<common::math::Vec2d>>&
        obstacles_vertices_vec) {
  if (obstacles_vertices_vec.empty()) {
    ADEBUG("No obstacles");
    return MarkerArray();
  }

  MarkerArray boundingboxs;

  const size_t num_of_obstacles = obstacles_vertices_vec.size();

  for (size_t i = 0; i < num_of_obstacles; i++) {
    auto obox = ObstacleBox(obstacles_vertices_vec[i]);
    obox.id = i;
    boundingboxs.markers.emplace_back(std::move(obox));
  }

  return boundingboxs;
}

Marker TrajectoryVisualizer::ObstacleBox(
    const std::vector<common::math::Vec2d>& obstacle_vertices) {
  Marker obstacle_box;

  obstacle_box.type = Marker::LINE_STRIP;
  obstacle_box.header.frame_id = frame_id_;
  obstacle_box.ns = "obstacles";
  obstacle_box.action = Marker::ADD;
  obstacle_box.color = properties_["obstacles"].first;
  obstacle_box.scale = properties_["obstacles"].second;
  if (!obstacle_vertices.empty()) {
    for (const auto& vertice : obstacle_vertices) {
      auto point = ToGeometryPose(vertice.x(), vertice.y(), 0.0).position;
      obstacle_box.points.emplace_back(point);
    }
    obstacle_box.points.emplace_back(
        ToGeometryPose(obstacle_vertices.front().x(),
                       obstacle_vertices.front().y(), 0.0)
            .position);
  }
  return obstacle_box;
}

}  // namespace util
}  // namespace common
}  // namespace autoagric