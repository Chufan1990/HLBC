#pragma once

#include <ros/ros.h>

#include <memory>
#include <utility>
#include <vector>

#include "autoagric/planning/planning.pb.h"
#include "autoware_msgs/Lane.h"
#include "geometry_msgs/Vector3.h"
#include "hlbc/Trajectory.h"
#include "std_msgs/ColorRGBA.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "visualization_msgs/MarkerArray.h"

namespace autoagric {
namespace control {

class TrajectoryVisualizer {
 public:
  typedef std::pair<visualization_msgs::MarkerArray,
                    visualization_msgs::MarkerArray>
      MarkerType;
  TrajectoryVisualizer(ros::NodeHandle& nh);

  TrajectoryVisualizer(ros::NodeHandle& nh,
                       const std::vector<std::string>& names);

  TrajectoryVisualizer() = default;

  virtual ~TrajectoryVisualizer();

  void Init();

  void Proc(const std::vector<MarkerType>& markers);

  static MarkerType toMarkerArray(const hlbc::Trajectory& lane,
                                  const geometry_msgs::Vector3& scale,
                                  const std_msgs::ColorRGBA& color);

  static MarkerType toMarkerArray(const autoware_msgs::Lane& lane,
                                  const geometry_msgs::Vector3& scale,
                                  const std_msgs::ColorRGBA& color);

  static MarkerType toMarkerArray(const std::vector<double>& xs,
                                  const std::vector<double>& ys,
                                  const std::vector<double>& headings,
                                  const std_msgs::Header& header,
                                  const geometry_msgs::Vector3& scale,
                                  const std_msgs::ColorRGBA& color);

  template <typename T>
  static MarkerType TrajectoryToMarkerArray(const T& trajectory,
                                            const std::string& frame_id,
                                            const double timestamp,
                                            const geometry_msgs::Vector3& scale,
                                            const std_msgs::ColorRGBA& color) {
    visualization_msgs::Marker points, line_strip;
    visualization_msgs::MarkerArray arrows, points_and_line;

    points.header.frame_id = line_strip.header.frame_id = frame_id;
    points.header.stamp.sec = line_strip.header.stamp.sec = timestamp;
    points.ns = "points";
    line_strip.ns = "line";
    points.id = 0;
    line_strip.id = 1;

    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;

    points.scale = line_strip.scale = scale;
    points.color = line_strip.color = color;
    // points.lifetime = line_strip.lifetime = ros::Duration(0);

    int i = 0;
    for (auto& point : trajectory) {
      geometry_msgs::Point p;
      p.x = point.path_point().x();
      p.y = point.path_point().y();
      p.z = point.path_point().z();

      points.points.push_back(p);
      line_strip.points.push_back(p);

      visualization_msgs::Marker arrow;
      arrow.pose.position.x = point.path_point().x();
      arrow.pose.position.y = point.path_point().y();
      arrow.pose.position.z = point.path_point().z();

      auto q = tf2::Quaternion();
      q.setRPY(0, 0, point.path_point().theta());
      arrow.pose.orientation = tf2::toMsg(q);

      arrow.header.frame_id = frame_id;
      arrow.id = i++;
      arrow.header.stamp.sec = timestamp;
      arrow.type = visualization_msgs::Marker::ARROW;
      arrow.scale = scale;
      arrow.color = color;
      // arrow.lifetime = ros::Duration(0);
      arrows.markers.push_back(arrow);
    }

    points_and_line.markers.push_back(points);
    points_and_line.markers.push_back(line_strip);

    return std::make_pair(arrows, points_and_line);
  }

 private:
  MarkerType toMarkerArray(const autoware_msgs::Lane& lane);

  std::vector<ros::NodeHandle> nh_queue_;

  std::vector<std::pair<ros::Publisher, ros::Publisher>> publisher_queue_;
};

}  // namespace control
}  // namespace autoagric
