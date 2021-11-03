#pragma once

#include <ros/ros.h>

#include <utility>

#include "autoware_msgs/Lane.h"
#include "visualization_msgs/MarkerArray.h"

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

 private:
  MarkerType LaneToMarkerArray(const autoware_msgs::LaneConstPtr& lane);

  ros::NodeHandle nh_;

  ros::Subscriber sub1_;
  ros::Publisher pub1_, pub2_;
};

}  // namespace common
}  // namespace control
}  // namespace autoagric
