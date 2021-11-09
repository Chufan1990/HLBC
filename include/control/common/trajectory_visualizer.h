#pragma once

#include <ros/ros.h>

#include <utility>

#include "autoware_msgs/Lane.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/ColorRGBA.h"
#include "hlbc/proto/planning.pb.h"



namespace autoagric {
namespace control {
namespace common {

class TrajectoryVisualizer {
 public:
  typedef std::pair<visualization_msgs::MarkerArray,
                    visualization_msgs::MarkerArray>
      MarkerType;
  TrajectoryVisualizer(ros::NodeHandle& nh);

  virtual ~TrajectoryVisualizer() = default;

  void Init();

  void Proc(const autoware_msgs::LaneConstPtr& lane);


  static MarkerType LaneToMarkerArray(const autoware_msgs::Lane& lane, const geometry_msgs::Vector3& scale,
      const std_msgs::ColorRGBA& color);


  static MarkerType ADCTrajectoryToMarkerArray(
    const planning::ADCTrajectory& trajectory, const geometry_msgs::Vector3& scale,
    const std_msgs::ColorRGBA& color);

 private:
  MarkerType LaneToMarkerArray(const autoware_msgs::LaneConstPtr& lane);

  ros::NodeHandle nh_;

  ros::Subscriber sub1_;
  ros::Publisher pub1_, pub2_;
};

}  // namespace common
}  // namespace control
}  // namespace autoagric
